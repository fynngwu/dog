#include "input_source.hpp"
#include <iostream>
#include "gamepad_command_source.hpp"
#include "keyboard_command_source.hpp"
#include "robot_config.hpp"

// ROS2 支持 (条件编译)
#if defined(ROS2_FOUND)
#include "ros2_command_source.hpp"
#endif

std::shared_ptr<ICommandSource> CreateInputSource(const std::string& input_mode) {
    if (input_mode == "gamepad") {
        auto gamepad = std::make_shared<Gamepad>(cfg::kGamepadDevice.c_str());
        return std::make_shared<GamepadCommandSource>(gamepad);

    } else if (input_mode == "keyboard") {
        return std::make_shared<KeyboardCommandSource>(0.5f, 0.0f, 0.5f);

    } else if (input_mode == "ros2") {
#ifdef ROS2_FOUND
        return std::make_shared<Ros2CommandSource>(
            cfg::kRos2CmdTopic,
            cfg::kRos2CmdVxScale,
            cfg::kRos2CmdVyScale,
            cfg::kRos2CmdYawScale,
            cfg::kRos2CmdDeadmanMs);
#else
        std::cerr << "[Input] ERROR: ROS2 input not available (compiled without ROS2 support)" << std::endl;
        std::cerr << "[Input] Falling back to keyboard" << std::endl;
        return std::make_shared<KeyboardCommandSource>(0.5f, 0.0f, 0.5f);
#endif

    } else {  // auto
        auto gamepad = std::make_shared<Gamepad>(cfg::kGamepadDevice.c_str());
        if (gamepad->IsConnected()) {
            std::cout << "[Input] Gamepad detected, using gamepad" << std::endl;
            return std::make_shared<GamepadCommandSource>(gamepad);
        } else {
            std::cout << "[Input] No gamepad, using keyboard" << std::endl;
            return std::make_shared<KeyboardCommandSource>(0.5f, 0.0f, 0.5f);
        }
    }
}
