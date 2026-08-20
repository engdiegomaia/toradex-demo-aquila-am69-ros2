//
// Created by tlab-uav on 24-9-6.
//

#include "unitree_guide_controller/UnitreeGuideController.h"

#include <stdexcept>
#include <string>

#include <unitree_guide_controller/gait/WaveGenerator.h>
#include "unitree_guide_controller/robot/QuadrupedRobot.h"

namespace unitree_guide_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration UnitreeGuideController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration UnitreeGuideController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type UnitreeGuideController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // auto now = std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_diff = now - last_update_time_;
        // last_update_time_ = now;
        //
        // // Calculate the frequency
        // update_frequency_ = 1.0 / time_diff.count();
        // RCLCPP_INFO(get_node()->get_logger(), "Update frequency: %f Hz", update_frequency_);
        if (ctrl_component_.robot_model_ == nullptr)
        {
            return controller_interface::return_type::OK;
        }

        ctrl_component_.robot_model_->update();
        ctrl_component_.wave_generator_->update();
        ctrl_component_.estimator_->update();

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            const auto &inputs = ctrl_interfaces_.control_inputs_;
            const auto &contact = ctrl_component_.wave_generator_->contact_;
            RCLCPP_INFO_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 1000,
                "gait diagnostics: state=%s command=%d sticks=(lx=%.4f ly=%.4f rx=%.4f) "
                "contact=[%d %d %d %d]",
                current_state_->state_name_string.c_str(), inputs.command,
                inputs.lx, inputs.ly, inputs.rx,
                contact(0), contact(1), contact(2), contact(3));
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            feet_names_ =
                auto_declare<std::vector<std::string>>("feet_names", feet_names_);

            // pose parameters
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);

            declareGaitParams();

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_);
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                // Handle message
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                    ctrl_interfaces_, msg->data, feet_names_, base_name_);
                ctrl_component_.balance_ctrl_ = std::make_shared<BalanceCtrl>(
                    ctrl_component_.robot_model_, ctrl_component_.gait_params_);
            });

        // The bias stays a literal: (0, 0.5, 0.5, 0) is what makes this a trot --
        // FR/RL against FL/RR -- rather than a tuning knob.  Period and stance
        // ratio are tuning and come from parameters.
        const auto &gait = ctrl_component_.gait_params_;
        ctrl_component_.wave_generator_ = std::make_shared<WaveGenerator>(
            gait.gait_period, gait.gait_stance_ratio, Vec4(0, 0.5, 0.5, 0));

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.swingTest = std::make_shared<StateSwingTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_);
        state_list_.balanceTest = std::make_shared<StateBalanceTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.trotting = std::make_shared<StateTrotting>(ctrl_interfaces_, ctrl_component_);

        // Initialize FSM
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    void UnitreeGuideController::declareGaitParams()
    {
        auto &gait = ctrl_component_.gait_params_;

        // Fixed-length gains read as lists, with the length checked.  A list of
        // the wrong length would otherwise be accepted as a partially-defaulted
        // gain matrix: the controller comes up, the robot walks differently, and
        // no log line says why.  Throwing here surfaces in on_init's handler.
        auto declare_vec = [this](const std::string &name, const auto &fallback)
        {
            using VecT = std::decay_t<decltype(fallback)>;
            constexpr int rows = VecT::RowsAtCompileTime;

            const std::vector<double> defaults(fallback.data(), fallback.data() + rows);
            const auto read = auto_declare<std::vector<double>>(name, defaults);
            if (read.size() != static_cast<size_t>(rows))
            {
                throw std::invalid_argument(
                    name + " needs exactly " + std::to_string(rows) + " values, got " +
                    std::to_string(read.size()));
            }

            VecT value;
            for (int i = 0; i < rows; ++i)
            {
                value(i) = read[i];
            }
            return value;
        };

        gait.gait_period = auto_declare<double>("gait.period", gait.gait_period);
        gait.gait_stance_ratio = auto_declare<double>("gait.stance_ratio", gait.gait_stance_ratio);
        gait.gait_height = auto_declare<double>("gait.height", gait.gait_height);

        gait.k_x = auto_declare<double>("foot_placement.k_x", gait.k_x);
        gait.k_y = auto_declare<double>("foot_placement.k_y", gait.k_y);
        gait.k_yaw = auto_declare<double>("foot_placement.k_yaw", gait.k_yaw);

        gait.kp_p = declare_vec("trot.kp_p", gait.kp_p);
        gait.kd_p = declare_vec("trot.kd_p", gait.kd_p);
        gait.kp_w = auto_declare<double>("trot.kp_w", gait.kp_w);
        gait.kd_w = declare_vec("trot.kd_w", gait.kd_w);
        gait.kp_swing = declare_vec("trot.kp_swing", gait.kp_swing);
        gait.kd_swing = declare_vec("trot.kd_swing", gait.kd_swing);

        gait.v_x_limit = declare_vec("trot.v_x_limit", gait.v_x_limit);
        gait.v_y_limit = declare_vec("trot.v_y_limit", gait.v_y_limit);
        gait.w_yaw_limit = declare_vec("trot.w_yaw_limit", gait.w_yaw_limit);
        gait.reference_band = auto_declare<double>("trot.reference_band", gait.reference_band);

        gait.acc_limit_xy = auto_declare<double>("trot.acc_limit_xy", gait.acc_limit_xy);
        gait.acc_limit_z = auto_declare<double>("trot.acc_limit_z", gait.acc_limit_z);
        gait.ang_acc_limit_roll_pitch =
            auto_declare<double>("trot.ang_acc_limit_roll_pitch", gait.ang_acc_limit_roll_pitch);
        gait.ang_acc_limit_yaw =
            auto_declare<double>("trot.ang_acc_limit_yaw", gait.ang_acc_limit_yaw);

        gait.weight_force = declare_vec("balance.weight_force", gait.weight_force);
        gait.weight_moment = declare_vec("balance.weight_moment", gait.weight_moment);
        gait.friction_ratio = auto_declare<double>("balance.friction_ratio", gait.friction_ratio);

        gait.hold_weight_moment_yaw =
            auto_declare<double>("hold.weight_moment_yaw", gait.hold_weight_moment_yaw);
        gait.hold_settle_rate = auto_declare<double>("hold.settle_rate", gait.hold_settle_rate);

        // WaveGenerator answers a bad period or stance ratio with exit(-1),
        // which takes the whole controller_manager process down -- acceptable
        // for a compiled-in literal, not for a value someone can now type into
        // a YAML file.  Rejected here instead, while a failure is still just a
        // controller that refuses to load.
        if (gait.gait_period <= 0.0)
        {
            throw std::invalid_argument(
                "gait.period must be positive, got " + std::to_string(gait.gait_period));
        }
        if (gait.gait_stance_ratio <= 0.0 || gait.gait_stance_ratio >= 1.0)
        {
            throw std::invalid_argument(
                "gait.stance_ratio must be in (0, 1), got " + std::to_string(gait.gait_stance_ratio));
        }
        if (gait.friction_ratio <= 0.0)
        {
            throw std::invalid_argument(
                "balance.friction_ratio must be positive, got " + std::to_string(gait.friction_ratio));
        }

        RCLCPP_INFO(get_node()->get_logger(),
                    "gait params: period=%.3f st_ratio=%.3f height=%.3f "
                    "k=(%.4f %.4f %.4f) kp_w=%.1f kd_w=(%.1f %.1f %.1f) "
                    "yaw_clamp=%.1f band=%.4f "
                    "S_moment=(%.0f %.0f %.0f) mu=%.2f "
                    "hold=(Syaw %.0f, settle %.3f m/s)",
                    gait.gait_period, gait.gait_stance_ratio, gait.gait_height,
                    gait.k_x, gait.k_y, gait.k_yaw, gait.kp_w,
                    gait.kd_w(0), gait.kd_w(1), gait.kd_w(2),
                    gait.ang_acc_limit_yaw, gait.reference_band,
                    gait.weight_moment(0), gait.weight_moment(1), gait.weight_moment(2),
                    gait.friction_ratio,
                    gait.hold_weight_moment_yaw, gait.hold_settle_rate);
    }

    std::shared_ptr<FSMState> UnitreeGuideController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::FIXEDSTAND:
            return state_list_.fixedStand;
        case FSMStateName::FREESTAND:
            return state_list_.freeStand;
        case FSMStateName::TROTTING:
            return state_list_.trotting;
        case FSMStateName::SWINGTEST:
            return state_list_.swingTest;
        case FSMStateName::BALANCETEST:
            return state_list_.balanceTest;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_guide_controller::UnitreeGuideController, controller_interface::ControllerInterface);
