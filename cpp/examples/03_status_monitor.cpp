// Example 3: Continuously print gripper state changes.
//
// Connects to the gripper, prints its firmware version and serial number
// (firmware >= v1.25 / v1.24), then polls state every 100 ms. Prints a line
// each time something interesting changes (position, current, object
// state, or fault).  CTRL+C to stop.
//
// Usage:
//   ./03_status_monitor /dev/ttyUSB0
//   03_status_monitor.exe COM3
#include "aidin/gripper.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop.store(true); }
}

int main(int argc, char** argv) {
    const std::string port = (argc > 1) ? argv[1]
#if defined(_WIN32)
                                        : "COM3";
#else
                                        : "/dev/ttyUSB0";
#endif

    std::signal(SIGINT, onSignal);

    try {
        aidin::Gripper g;
        g.connect(port);
        const std::string serial = g.readSerialNumber();
        std::cout << "Firmware " << g.readFirmwareInfo()
                  << ", serial " << (serial.empty() ? "(not recorded)" : serial) << "\n";
        std::cout << "Monitoring " << port << " (CTRL+C to quit)\n";

        aidin::GripperState prev{};
        bool first = true;
        while (!g_stop.load()) {
            const auto s = g.readState();
            const bool changed =
                first ||
                s.position != prev.position ||
                s.object   != prev.object   ||
                s.fault    != prev.fault    ||
                (std::abs(int(s.current) - int(prev.current)) > 5);
            if (changed) {
                std::cout << s << "\n";
                prev  = s;
                first = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Stopped.\n";
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
