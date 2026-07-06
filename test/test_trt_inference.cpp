// Smoke test for the TRT inference wrapper.
//   1. ConvertONNXToTRT(...) → build/cache the .trt engine next to the .onnx.
//   2. TRTInferenceEngine::Initialize(...) → load the cached engine.
//   3. Query every input / output tensor's name, shape, and dtype.
//   4. Feed zero-filled buffers, run one inference, print output shape.
//
// Run inside the dev container:
//   ./docker/run_dev.sh
//   # then, inside:
//   cmake -B build && cmake --build build
//   ./build/test_trt_inference models/planner_sonic.onnx

#include "tensorrt/InferenceEngine.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

static const char* dtype_name(DataType t) {
    switch (t) {
        case DataType::FLOAT:   return "float32";
        case DataType::HALF:    return "float16";
        case DataType::INT8:    return "int8";
        case DataType::INT32:   return "int32";
        case DataType::BOOL:    return "bool";
        case DataType::UINT8:   return "uint8";
        case DataType::INT64:   return "int64";
        case DataType::UNKNOWN: return "unknown";
    }
    return "?";
}

static size_t dtype_size(DataType t) {
    switch (t) {
        case DataType::FLOAT: return 4;
        case DataType::HALF:  return 2;
        case DataType::INT8:  return 1;
        case DataType::INT32: return 4;
        case DataType::BOOL:  return 1;
        case DataType::UINT8: return 1;
        case DataType::INT64: return 8;
        default:              return 0;
    }
}

static void print_shape(const std::vector<int64_t>& shape) {
    std::cout << "[";
    for (size_t i = 0; i < shape.size(); ++i) {
        std::cout << shape[i];
        if (i + 1 < shape.size()) std::cout << ", ";
    }
    std::cout << "]";
}

static size_t num_elements(const std::vector<int64_t>& shape) {
    return std::accumulate(shape.begin(), shape.end(), size_t{1},
                           [](size_t a, int64_t b) { return a * static_cast<size_t>(b); });
}

int main(int argc, char** argv) {
    const std::string onnx_path = (argc >= 2) ? argv[1] : "models/planner_sonic.onnx";

    // ── 1. Convert ONNX → TRT (cached) ────────────────────────────────────────
    Options opts;
    opts.precision = Precision::FP16;
    opts.deviceID  = 0;

    std::string trt_path;
    std::cout << "[Convert] " << onnx_path << "\n";
    if (!ConvertONNXToTRT(opts, onnx_path, trt_path)) {
        std::cerr << "ConvertONNXToTRT failed\n";
        return 1;
    }
    std::cout << "[Convert] engine: " << trt_path << "\n\n";

    // ── 2. Load engine ────────────────────────────────────────────────────────
    TRTInferenceEngine engine;
    if (!engine.Initialize(trt_path, 0)) {
        std::cerr << "Engine Initialize failed\n";
        return 1;
    }
    if (!engine.InitInputs()) {
        std::cerr << "Engine InitInputs failed\n";
        return 1;
    }

    // ── 3. Discover schema ────────────────────────────────────────────────────
    auto in_names  = engine.GetInputTensorNames();
    auto out_names = engine.GetOutputTensorNames();

    std::cout << "Inputs:\n";
    for (const auto& name : in_names) {
        std::vector<int64_t> shape;
        engine.GetTensorShape(name, shape);
        std::cout << "  " << name << "  " << dtype_name(engine.GetTensorDataType(name)) << "  ";
        print_shape(shape);
        std::cout << "\n";
    }
    std::cout << "\nOutputs:\n";
    for (const auto& name : out_names) {
        std::vector<int64_t> shape;
        engine.GetTensorShape(name, shape);
        std::cout << "  " << name << "  " << dtype_name(engine.GetTensorDataType(name)) << "  ";
        print_shape(shape);
        std::cout << "\n";
    }
    std::cout << "\n";

    // ── 4. Feed zero-filled inputs, run one inference ─────────────────────────
    for (const auto& name : in_names) {
        std::vector<int64_t> shape;
        engine.GetTensorShape(name, shape);
        size_t bytes = num_elements(shape) * dtype_size(engine.GetTensorDataType(name));
        std::vector<std::byte> zeros(bytes, std::byte{0});
        engine.SetInputData(name, zeros.data(), bytes);
    }

    std::cout << "[Run] Enqueue()\n";
    if (!engine.Enqueue(nullptr)) {
        std::cerr << "Enqueue failed\n";
        return 1;
    }

    for (const auto& name : out_names) {
        std::vector<int64_t> shape;
        engine.GetTensorShape(name, shape);
        size_t bytes = num_elements(shape) * dtype_size(engine.GetTensorDataType(name));
        std::vector<std::byte> out(bytes);
        engine.GetOutputData(name, out.data(), bytes);
        std::cout << "[Output] " << name << " received " << bytes << " bytes\n";
    }

    std::cout << "\n[Done]\n";
    return 0;
}
