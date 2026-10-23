// Example 5: High-rate polling with the fast block (0x02C0..0x02C9).
//
// One FC03 read of 10 registers returns gripper status, position, current,
// fault AND both fingertips' tactile resultant as one consistent snapshot
// (~4 ms per read at 115200 bps). Tactile values are 8-bit compressed over
// each field's full scale (Px 32 mm, Py +-12.5 mm, Fx/Fy +-35 N, Fz 70 N).
// Needs firmware protocol >= v1.19.0 for the tactile scales; tactile fields
// are 0 unless the sensor type is tactile.
//
// Usage:
//   ./05_fast_poll /dev/ttyUSB0
//   05_fast_poll.exe COM3
#include "aidin/gripper.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>

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
        std::cout << "Polling " << port << " (CTRL+C to quit)\n";

        using Clock = std::chrono::steady_clock;
        int  n  = 0;
        auto t0 = Clock::now();
        while (!g_stop.load()) {
            const auto s = g.readFastState();
            ++n;
            const auto now = Clock::now();
            const double dt = std::chrono::duration<double>(now - t0).count();
            if (dt >= 0.5) {                         // print twice a second, keep polling flat out
                std::cout << std::fixed << std::setprecision(1) << std::setw(6) << n / dt << " Hz  "
                          << s << "    \r" << std::flush;
                n  = 0;
                t0 = now;
            }
        }
        std::cout << "\nStopped.\n";
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
