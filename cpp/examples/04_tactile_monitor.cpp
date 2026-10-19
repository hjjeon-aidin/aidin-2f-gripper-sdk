// Example 4: Stream the 3-axis fingertip tactile sensors.
//
// Each fingertip reports one contact resultant at 100 Hz: contact point
// (posX, posY) [mm] and force (fx, fy, fz) [N]. Tip 1 = right finger, tip 2 =
// left finger. Needs firmware protocol >= v1.18.0 and CFG_SENSOR_TYPE = 2
// (tactile) — see "Tactile sensor" in MANUAL.md for the one-time setup.
//
// Usage:
//   ./04_tactile_monitor /dev/ttyUSB0          # stream both tips @ 20 Hz
//   ./04_tactile_monitor /dev/ttyUSB0 --zero   # sensor no-load zero first
//   04_tactile_monitor.exe COM3
#include "aidin/gripper.hpp"
#include "aidin/registers.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop.store(true); }

// Wait until every detected tip is Live (sensor boot zero takes ~4.5 s).
void waitLive(aidin::Gripper& g, std::chrono::milliseconds timeout = std::chrono::seconds(6)) {
    using aidin::TactileLink;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto tips = g.readTactile();
        bool ready = true;
        for (const auto& t : tips) {
            if (t.link() != TactileLink::Live && t.link() != TactileLink::NotDetected) ready = false;
        }
        if (ready) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    const auto tips = g.readTactile();
    std::cout << "right: " << aidin::toString(tips[0].link())
              << "   left: " << aidin::toString(tips[1].link()) << "\n";
}
}

int main(int argc, char** argv) {
    std::string port =
#if defined(_WIN32)
        "COM3";
#else
        "/dev/ttyUSB0";
#endif
    bool zero = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--zero") == 0) zero = true;
        else port = argv[i];
    }

    std::signal(SIGINT, onSignal);

    try {
        aidin::Gripper g;
        g.connect(port);

        const uint16_t sensor = g.readRegister(aidin::reg::CFG_SENSOR_TYPE);
        if (sensor != aidin::reg::SENSOR_TACTILE) {
            std::cerr << "CFG_SENSOR_TYPE = " << sensor << ", not tactile (2). Write 2 to 0x018A, "
                         "DEV_CMD 0x02 SAVE_CONFIG, then power-cycle.\n";
            return 1;
        }

        waitLive(g);

        if (zero) {
            std::cout << "Zeroing - keep the pads untouched...\n";
            g.tactileZero();                         // both tips; Waiting ~1.25 s, then Live
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            waitLive(g);
        }

        std::cout << "Streaming (CTRL+C to quit). pos [mm], F [N]\n";
        while (!g_stop.load()) {
            const auto tips = g.readTactile();       // {right, left}
            std::cout << "R " << tips[0] << "  |  L " << tips[1] << "        \r" << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::cout << "\nStopped.\n";
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
