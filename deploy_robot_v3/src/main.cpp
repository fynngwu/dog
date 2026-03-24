#include "minimal_dog.hpp"
#include <iostream>
#include <signal.h>
#include <cstdlib>
#include <atomic>

using minimal::MinimalDog;

// Global flag for signal handler (non-static for use in minimal_dog.cpp)
std::atomic<bool> g_running{true};

void signal_handler(int sig) {
    (void)sig;  // Suppress unused warning
    g_running = false;
}

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --help, -h     Show this help message" << std::endl;
    std::cout << "  --time <sec>   Run for specified duration (default: infinite)" << std::endl;
    std::cout << std::endl;
    std::cout << "ROS2: Publish to /cmd_vel topic to control the robot." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    float duration_sec = 0.0f;  // 0 = infinite

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--time" && i + 1 < argc) {
            duration_sec = std::atof(argv[++i]);
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "   Minimal Dog Controller (deploy_v3)   " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Duration: " << (duration_sec > 0 ? std::to_string(duration_sec) + "s" : "infinite") << std::endl;
    std::cout << std::endl;

    // Create and initialize robot
    MinimalDog dog;

    if (!dog.Initialize()) {
        std::cerr << "Initialization failed!" << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "=== Ready to run! ===" << std::endl;
    std::cout << "Press Ctrl+C to stop..." << std::endl;
    std::cout << std::endl;

    // Run main control loop (destructor will call Stop)
    dog.Run(duration_sec);

    std::cout << "=== Done ===" << std::endl;
    return 0;
}
