#include "my_bot/arduino_hardware.hpp"
#include <sstream>
#include "pluginlib/class_list_macros.hpp"

namespace my_bot{
    hardware_interface::CallbackReturn MyBotArduinoHardware::on_init(
        const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
        device_=info_.hardware_parameters["device"];
        timeout_ms_=std::stoi(info_.hardware_parameters["timeout_ms"]);
        enc_counts_per_rev_=std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MyBotArduinoHardware::on_configure(const rclcpp_lifecycle::State & previous_state){

        try{
            serial_port_.Open(device_);
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            return CallbackReturn::SUCCESS;
        }
        catch(const std::exception & ERROR){ 
            return CallbackReturn::ERROR;
        }
    }

    hardware_interface::CallbackReturn MyBotArduinoHardware::on_activate(const rclcpp_lifecycle::State & previous_state){
        try{
                serial_port_.Write("\r");
                return CallbackReturn::SUCCESS;
            }
            catch(const std::exception & ERROR){ 
                return CallbackReturn::ERROR;
            }

    }

    hardware_interface::CallbackReturn MyBotArduinoHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state){
        try{
            serial_port_.Close();
            return CallbackReturn::SUCCESS;
        }
        catch(const std::exception & ERROR){ 
            return CallbackReturn::ERROR;
        }
        
    }
    std::vector<hardware_interface::StateInterface> MyBotArduinoHardware::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(
            "left_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &wheel_left_.velocity);
        state_interfaces.emplace_back(
            "right_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &wheel_right_.velocity);
        state_interfaces.emplace_back(
            "left_wheel_joint",
            hardware_interface::HW_IF_POSITION,
            &wheel_left_.position);
        state_interfaces.emplace_back(
            "right_wheel_joint",
            hardware_interface::HW_IF_POSITION,
            &wheel_right_.position);
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MyBotArduinoHardware::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(
            "left_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &wheel_left_.command);
        command_interfaces.emplace_back(
            "right_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &wheel_right_.command);
        return command_interfaces;

    }
}





hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

PLUGINLIB_EXPORT_CLASS(
    my_bot::MyBotArduinoHardware,
    hardware_interface::SystemInterface
)