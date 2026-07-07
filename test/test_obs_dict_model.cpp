// Smoke test for the encoder/decoder TRT wrappers.
//   - Loads model_encoder.onnx (obs_dict[1762] -> encoded_tokens[64]) and
//     model_decoder.onnx (obs_dict[994] -> action[29]).
//   - Runs both on a plausible zero-state input and prints outputs.
//   - Chains encoder tokens into the decoder obs slot [0:64] the way the
//     control loop will.

#include "common/config.hpp"
#include "control/obs_dict_model.hpp"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(config_path);
    auto control_cfg  = kist::Config::instance().root()["control"];
    auto encoder_path = control_cfg["encoder_path"].as<std::string>("models/model_encoder.onnx");
    auto decoder_path = control_cfg["decoder_path"].as<std::string>("models/model_decoder.onnx");

    kist::ObsDictModel encoder, decoder;
    if (!encoder.Initialize(encoder_path, "encoded_tokens", 1762, 64)) {
        std::cerr << "encoder init failed\n";
        return 1;
    }
    if (!decoder.Initialize(decoder_path, "action", 994, 29)) {
        std::cerr << "decoder init failed\n";
        return 1;
    }

    // Encoder input: zeros (g1 mode 0 -> encoder_mode_4 slot is also 0)
    std::memset(encoder.input(), 0, encoder.input_dim() * sizeof(float));
    if (!encoder.Infer()) return 1;

    std::cout << std::fixed << std::setprecision(4) << "tokens[0:8] = ";
    for (int i = 0; i < 8; ++i) std::cout << encoder.output()[i] << " ";
    std::cout << "\n";

    // Decoder input: tokens at [0:64], rest zeros
    std::memset(decoder.input(), 0, decoder.input_dim() * sizeof(float));
    std::memcpy(decoder.input(), encoder.output(), 64 * sizeof(float));
    if (!decoder.Infer()) return 1;

    std::cout << "action[0:8] = ";
    for (int i = 0; i < 8; ++i) std::cout << decoder.output()[i] << " ";
    std::cout << "\n";

    // Rough timing: 100 chained iterations
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; ++i) {
        encoder.Infer();
        std::memcpy(decoder.input(), encoder.output(), 64 * sizeof(float));
        decoder.Infer();
    }
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - t0).count();
    std::cout << "encoder+decoder chained: " << us / 100.0 << " us/tick (budget 20000)\n";
    return 0;
}
