# XRoboToolkit PC daemon (PICO VR). Keep LD_LIBRARY_PATH scoped to the daemon:
# its lib dir has private Qt6/openssl copies that break other binaries if exported.
run_vr_daemon() {
    LD_LIBRARY_PATH=/opt/apps/roboticsservice:/opt/apps/roboticsservice/lib \
        setsid /opt/apps/roboticsservice/RoboticsServiceProcess "$@" \
        >> /tmp/roboticsservice.log 2>&1 < /dev/null &
    echo "RoboticsService started (log: /tmp/roboticsservice.log)"
}
