# Helper for running the XRoboToolkit PC daemon (PICO VR).
#
# Our own binaries need no LD_LIBRARY_PATH setup: CMake bakes the SDK
# library locations into each executable's RUNPATH at build time.
# The daemon's deb ships its libraries (private Qt6/openssl copies)
# without rpath, so the path is scoped to the daemon process only.
run_vr_daemon() {
    LD_LIBRARY_PATH=/opt/apps/roboticsservice:/opt/apps/roboticsservice/lib \
        /opt/apps/roboticsservice/RoboticsServiceProcess "$@" &
}
