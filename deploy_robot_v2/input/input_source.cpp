#include "input_source.hpp"
#include <iostream>
#include "gamepad_command_source.hpp"
#include "keyboard_command_source.hpp"
#include "robot_config.hpp"

std::shared_ptr<ICommandSource> CreateInputSource(const std::string& input_mode) {
    if (input_mode == "gamepad") {
        auto gamepad = std::make_shared<Gamepad>(cfg::kGamepadDevice.c_str());
        return std::make_shared<GamepadCommandSource>(gamepad);
    } else if (input_mode == "keyboard") {
        return std::make_shared<KeyboardCommandSource>(0.5f, 0.0f, 0.5f);
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
