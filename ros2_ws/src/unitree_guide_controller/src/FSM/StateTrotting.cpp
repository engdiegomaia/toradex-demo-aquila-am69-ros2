//
// Created by tlab-uav on 24-9-18.
//

#include "unitree_guide_controller/FSM/StateTrotting.h"

#include <algorithm>
#include <cmath>

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/control/CtrlComponent.h>
#include <unitree_guide_controller/control/Estimator.h>
#include <unitree_guide_controller/gait/WaveGenerator.h>

namespace {
    // Walk-intent thresholds, in SI units, applied where StateTrotting has
    // already turned stick axes into a body velocity command.  They are NOT the
    // upstream numbers, and the difference is arithmetic rather than taste:
    //
    //   Twist.linear.x = 0.01 m/s
    //     -> twist_to_inputs keeps unit gain and clamps the stick: ly = 0.01
    //     -> getUserCmd(): invNormalize(0.01, -0.4, 0.4) = 0.4 * 0.01 = 0.004 m/s
    //
    // So the slowest command in the F4 plan arrives here as 4 mm/s, while
    // upstream only starts stepping above 0.03 m/s — 7.5x more than this bridge
    // can ever produce with its 0.03 stick clamp.  That is why the recorded run
    // shows `state=trotting`, `ly=0.01..0.03` and `contact=[1 1 1 1]` for the
    // whole window (docs/results/ml35-f4-parcial.md): no step was ever
    // requested.  Tuning gains against that log would have measured nothing.
    //
    // Widening the stick envelope in twist_to_inputs changes this arithmetic;
    // revisit these numbers together with it, not separately.
    constexpr double V_START = 0.002; // m/s, fires for Twist.linear.x >= 0.005
    constexpr double V_STOP = 0.001; // m/s
    constexpr double W_START = 0.005; // rad/s, fires for Twist.angular.z >= 0.01
    constexpr double W_STOP = 0.002; // rad/s

    // Attitude supervision, radians of body-z tilt away from gravity.  Starting
    // points for simulation, not Go2 specifications: log the tilt column of the
    // diagnostics line across a run before moving them.
    //
    // Upstream only reacts through FSM::checkSafty(), around 60 deg, which is
    // well past the point where a quadruped can still recover by standing.
    constexpr double TILT_OK = 0.087; // 5 deg: attitude considered settled
    constexpr double TILT_DERATE = 0.140; // 8 deg: start fading the command out
    constexpr double TILT_RECOVER = 0.209; // 12 deg: cancel locomotion entirely
    constexpr double RECOVER_SETTLE_S = 0.3; // time below TILT_OK before HOLD

    // Entry settle: hold four feet down for this long after FIXEDSTAND hands
    // over, before any gait is allowed.  Without it, whether the robot starts
    // walking on the first tick depends on whether the velocity command
    // happened to arrive in the same message as the FSM start command -- and
    // the wave generator's phase is free-running since construction, so
    // starting on tick 1 can lift a diagonal pair at an arbitrary point of the
    // cycle, on a body that has not settled. That is not a tuning detail: it
    // made two unrelated experiments fall in the same 1 s, hiding what they
    // were supposed to measure.
    constexpr double ENTRY_SETTLE_S = 0.3;

    const char *modeName(const MotionMode mode) {
        switch (mode) {
            case MotionMode::WALK:
                return "WALK";
            case MotionMode::RECOVER:
                return "RECOVER";
            default:
                return "HOLD";
        }
    }
}

StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces,
                             CtrlComponent &ctrl_component) : FSMState(FSMStateName::TROTTING, "trotting",
                                                                       ctrl_interfaces),
                                                              estimator_(ctrl_component.estimator_),
                                                              robot_model_(ctrl_component.robot_model_),
                                                              balance_ctrl_(ctrl_component.balance_ctrl_),
                                                              wave_generator_(ctrl_component.wave_generator_),
                                                              gait_generator_(ctrl_component) {
    gait_height_ = 0.08;
    Kpp = Vec3(70, 70, 70).asDiagonal();
    Kdp = Vec3(10, 10, 10).asDiagonal();
    // Upstream values.  Splitting these per axis was tried, on the measured
    // grounds that kp_w_ = 780 saturates the yaw clamp at 0.73 degrees of
    // heading error and hands that axis to bang-bang: with Kp_yaw = 100 and
    // Kd_yaw = 20 the yaw demand does leave the clamp and the heading recovers
    // once mid-run (-19 deg back to -4 deg) instead of diverging monotonically.
    // It is kept out anyway because it did not make the robot walk longer
    // (5.5 s against 7 s), and the reason is in the same measurement: the QP
    // tops out near 5.3 N.m of yaw moment, so no attitude gain can regulate
    // heading here.  Heading is a foot-placement problem -- see k_yaw_ in
    // FeetEndCalc, which is where the fix belongs.
    kp_w_ = 780;
    Kd_w_ = Vec3(70, 70, 70).asDiagonal();
    Kp_swing_ = Vec3(400, 400, 400).asDiagonal();
    Kd_swing_ = Vec3(10, 10, 10).asDiagonal();

    v_x_limit_ << -0.4, 0.4;
    v_y_limit_ << -0.3, 0.3;
    w_yaw_limit_ << -0.5, 0.5;
    dt_ = 1.0 / ctrl_interfaces_.frequency_;
}

void StateTrotting::enter() {
    pcd_ = estimator_->getPosition();
    pcd_(2) = -estimator_->getFeetPos2Body()(2, 0);
    v_cmd_body_.setZero();
    yaw_cmd_ = estimator_->getYaw();
    Rd = rotz(yaw_cmd_);
    w_cmd_global_.setZero();

    // FIXEDSTAND hands over a standing robot with no command pending, which is
    // exactly the HOLD contract; the reference above is the captured one.
    mode_ = MotionMode::HOLD;
    walking_ = false;
    hold_captured_ = true;
    tilt_ = 0.0;
    settled_s_ = 0.0;
    entry_s_ = 0.0;
    diag_ticks_ = 0;
    d_yaw_cmd_ = 0.0;
    d_yaw_cmd_past_ = 0.0;

    ctrl_interfaces_.control_inputs_.command = 0;
    gait_generator_.restart();
}

void StateTrotting::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    pos_body_ = estimator_->getPosition();
    vel_body_ = estimator_->getVelocity();

    B2G_RotMat = estimator_->getRotation();
    G2B_RotMat = B2G_RotMat.transpose();

    getUserCmd();
    // Decide before integrating: a cancelled command must never reach pcd_.
    updateMotionMode();
    calcCmd();

    if (mode_ != MotionMode::WALK) {
        // calcCmd() saturates the velocity target against the *measured* body
        // velocity, so a body that is already sliding would re-create the
        // velocity target that was just cancelled.  Zero it after the fact.
        vel_target_.setZero();
        w_cmd_global_.setZero();
        captureBodyReference();
    }

    gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_global_(2), gait_height_);
    gait_generator_.generate(pos_feet_global_goal_, vel_feet_global_goal_);

    calcTau();
    calcQQd();

    // The wave generator does not switch every foot at once: it holds the
    // previous contact condition per leg until that leg can legally change,
    // so entering and leaving the gait mid-cycle is safe.
    wave_generator_->status_ = mode_ == MotionMode::WALK
                                   ? WaveStatus::WAVE_ALL
                                   : WaveStatus::STANCE_ALL;

    calcGain();
    logDiagnostics();
}

void StateTrotting::exit() {
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

FSMStateName StateTrotting::checkChange() {
    // A zero command means HOLD, not FIXEDSTAND.  Leaving TROTTING stays an
    // explicit operator decision, as upstream.
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::TROTTING;
    }
}

void StateTrotting::getUserCmd() {
    /* Movement */
    v_cmd_body_(0) = invNormalize(ctrl_interfaces_.control_inputs_.ly, v_x_limit_(0), v_x_limit_(1));
    v_cmd_body_(1) = -invNormalize(ctrl_interfaces_.control_inputs_.lx, v_y_limit_(0), v_y_limit_(1));
    v_cmd_body_(2) = 0;

    /* Turning */
    d_yaw_cmd_ = -invNormalize(ctrl_interfaces_.control_inputs_.rx, w_yaw_limit_(0), w_yaw_limit_(1));
    d_yaw_cmd_ = 0.9 * d_yaw_cmd_past_ + (1 - 0.9) * d_yaw_cmd_;
    d_yaw_cmd_past_ = d_yaw_cmd_;
}

void StateTrotting::updateMotionMode() {
    // Tilt of the body z axis away from gravity, straight off the rotation
    // matrix: acos(R(2,2)).  No Euler conversion and no gimbal edge case.
    const double r22 = B2G_RotMat(2, 2);
    tilt_ = std::acos(std::clamp(r22, -1.0, 1.0));

    settled_s_ = tilt_ < TILT_OK ? settled_s_ + dt_ : 0.0;

    // Deterministic entry: stand first, walk after.  See ENTRY_SETTLE_S.
    if (entry_s_ < ENTRY_SETTLE_S) {
        entry_s_ += dt_;
        mode_ = MotionMode::HOLD;
        walking_ = false;
        cancelCommand();
        return;
    }

    if (mode_ == MotionMode::RECOVER) {
        cancelCommand();
        if (settled_s_ >= RECOVER_SETTLE_S) {
            // Back to HOLD, never straight back to WALK: walking again needs a
            // fresh command crossing V_START/W_START.
            mode_ = MotionMode::HOLD;
            walking_ = false;
        }
        return;
    }

    if (tilt_ > TILT_RECOVER) {
        mode_ = MotionMode::RECOVER;
        walking_ = false;
        cancelCommand();
        return;
    }

    // Between TILT_DERATE and TILT_RECOVER the command fades out linearly, so
    // an increasingly tilted robot slows down before it is forced to stop.  The
    // faded command feeds the hysteresis below, which means the transition into
    // HOLD happens on its own instead of needing a second threshold.
    const double derate = std::clamp((TILT_RECOVER - tilt_) / (TILT_RECOVER - TILT_DERATE), 0.0, 1.0);
    v_cmd_body_ *= derate;
    d_yaw_cmd_ *= derate;
    d_yaw_cmd_past_ = d_yaw_cmd_;

    if (updateWalkIntent()) {
        mode_ = MotionMode::WALK;
        hold_captured_ = false;
    } else {
        mode_ = MotionMode::HOLD;
        cancelCommand();
    }
}

bool StateTrotting::updateWalkIntent() {
    const double v = std::hypot(v_cmd_body_(0), v_cmd_body_(1));
    const double w = std::fabs(d_yaw_cmd_);

    if (!walking_) {
        walking_ = v > V_START || w > W_START;
    } else if (v < V_STOP && w < W_STOP) {
        walking_ = false;
    }
    return walking_;
}

void StateTrotting::cancelCommand() {
    v_cmd_body_.setZero();
    d_yaw_cmd_ = 0.0;
    // The yaw command is low-pass filtered against its own past value; leaving
    // the past value behind would keep replaying the cancelled turn.
    d_yaw_cmd_past_ = 0.0;
}

void StateTrotting::captureBodyReference() {
    // HOLD parks the body: capture once, on the way out of WALK, then let
    // BalanceCtrl hold that point.  pcd_ is an *integrated* reference, so
    // without this it keeps its pre-stop offset (up to the 0.05 m saturation
    // band) and the QP keeps accelerating the body to close it — the robot
    // creeps after the command is already zero.
    //
    // RECOVER re-captures every tick on purpose: that leaves pos_error_ at
    // zero, so the horizontal term becomes pure velocity damping while the
    // attitude term does the levelling.
    if (mode_ != MotionMode::RECOVER && hold_captured_) {
        return;
    }

    pcd_(0) = pos_body_(0);
    pcd_(1) = pos_body_(1);
    yaw_cmd_ = estimator_->getYaw();
    Rd = rotz(yaw_cmd_);
    hold_captured_ = true;
}

void StateTrotting::calcCmd() {
    /* Movement */
    vel_target_ = B2G_RotMat * v_cmd_body_;

    vel_target_(0) =
            saturation(vel_target_(0), Vec2(vel_body_(0) - 0.2, vel_body_(0) + 0.2));
    vel_target_(1) =
            saturation(vel_target_(1), Vec2(vel_body_(1) - 0.2, vel_body_(1) + 0.2));

    // How far the integrated reference may run ahead of the body.  Upstream
    // allows 0.05 m, and with Kpp = 70 that is 3.5 m/s^2 of horizontal
    // acceleration -- a third of gravity -- demanded of a robot commanded to
    // walk at 4 mm/s.  Every recorded run shows posErrXY climbing to exactly
    // this limit and staying there: the trot does not propel the body, the
    // reference keeps integrating, pegs at the clamp, and the QP then pushes
    // flat out until the robot pitches over.  The clamp is the throttle on
    // that runaway, so it is sized to the command, not to the A1's tuning.
    constexpr double REFERENCE_BAND = 0.01;

    pcd_(0) = saturation(pcd_(0) + vel_target_(0) * dt_,
                         Vec2(pos_body_(0) - REFERENCE_BAND, pos_body_(0) + REFERENCE_BAND));
    pcd_(1) = saturation(pcd_(1) + vel_target_(1) * dt_,
                         Vec2(pos_body_(1) - REFERENCE_BAND, pos_body_(1) + REFERENCE_BAND));

    vel_target_(2) = 0;

    /* Turning */
    yaw_cmd_ = yaw_cmd_ + d_yaw_cmd_ * dt_;
    Rd = rotz(yaw_cmd_);
    w_cmd_global_(2) = d_yaw_cmd_;
}

void StateTrotting::calcTau() {
    pos_error_ = pcd_ - pos_body_;
    vel_error_ = vel_target_ - vel_body_;

    Vec3 dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
    Vec3 d_wbd = kp_w_ * rotMatToExp(Rd * G2B_RotMat) +
                 Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());

    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-3, 3));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-3, 3));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-5, 5));

    d_wbd(0) = saturation(d_wbd(0), Vec2(-40, 40));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-40, 40));
    // Upstream's yaw clamp, kept after measuring it.  A friction-cone estimate
    // suggests two diagonal feet could produce ~14 N.m of yaw moment, which
    // would allow 26 rad/s^2, but that assumes the whole tangential budget goes
    // to yaw; it also has to propel and balance the body.  Raising the clamp to
    // 25 was tried and is worse: the demand pinned at the new limit and the QP
    // fell behind it (13.2 asked, 5.8 delivered), and because the moment
    // residual carries weight 450 against 20/20/50 on force, chasing the
    // impossible moment wrecked the linear force distribution -- the robot
    // walked backwards and fell in 4 s instead of 7.  10 rad/s^2 is 5.3 N.m on
    // this robot, which the QP tracks to within 2%.
    d_wbd(2) = saturation(d_wbd(2), Vec2(-10, 10));

    const Vec34 pos_feet_body_global = estimator_->getFeetPos2Body();
    Vec34 force_feet_global =
            -balance_ctrl_->calF(dd_pcd, d_wbd, B2G_RotMat, pos_feet_body_global, wave_generator_->contact_);


    Vec34 pos_feet_global = estimator_->getFeetPos();
    Vec34 vel_feet_global = estimator_->getFeetVel();

    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            force_feet_global.col(i) =
                    Kp_swing_ * (pos_feet_global_goal_.col(i) - pos_feet_global.col(i)) +
                    Kd_swing_ * (vel_feet_global_goal_.col(i) - vel_feet_global.col(i));
        }
    }

    Vec34 force_feet_body_ = G2B_RotMat * force_feet_global;

    std::vector<KDL::JntArray> current_joints = robot_model_->current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_->getTorque(force_feet_body_.col(i), i);
        for (int j = 0; j < 3; j++) {
            std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
        }
    }
}

void StateTrotting::calcQQd() {
    const std::vector<KDL::Frame> pos_feet_body = robot_model_->getFeet2BPositions();

    Vec34 pos_feet_target, vel_feet_target;
    for (int i(0); i < 4; ++i) {
        pos_feet_target.col(i) = G2B_RotMat * (pos_feet_global_goal_.col(i) - pos_body_);
        vel_feet_target.col(i) = G2B_RotMat * (vel_feet_global_goal_.col(i) - vel_body_);
    }

    Vec12 q_goal = robot_model_->getQ(pos_feet_target);
    Vec12 qd_goal = robot_model_->getQd(pos_feet_body, vel_feet_target);
    for (int i = 0; i < 12; i++) {
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }
}

void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            // swing gain
            for (int j = 0; j < 3; j++) {
                std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(3.0);
                std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(2.0);
            }
        } else {
            // Keep the stance leg tracking at the same conservative PD gain
            // used by the swing leg.  The previous 0.8/0.8 values created a
            // large discontinuity when FIXEDSTAND (80/3.5) handed control to
            // TROTTING; even with zero velocity command the body then slowly
            // sagged.  This step changes only the gain discontinuity.  Gait
            // period, estimator and force controller remain untouched.
            for (int j = 0; j < 3; j++) {
                std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(3.0);
                std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(2.0);
            }
        }
    }
}

void StateTrotting::logDiagnostics() {
    // 4 Hz, not 1 Hz: the observed failure goes from a level body to a fallen
    // one inside a single second, so a 1 Hz line samples the collapse at most
    // once and cannot show its shape.
    const int period_ticks = std::max(1, ctrl_interfaces_.frequency_ / 4);
    if (++diag_ticks_ < period_ticks) {
        return;
    }
    diag_ticks_ = 0;

    // Everything needed to tell the four failure modes apart in one line:
    // no step requested, residual reference after stop, attitude loss, and
    // tracking error.  Reading `contact` alone cannot distinguish them.
    //
    // est* is the estimator's own belief about the body.  It is logged next to
    // the errors because every foot target is built from it -- calcFootPos()
    // places the foot at estimator position + radius * cos(estimator yaw + ...)
    // and then forces global z to 0.  If the estimator drifts, the whole
    // placement pattern translates and rotates with it while the tracking
    // errors, which are computed against the same drifting belief, stay small.
    // Comparing this against /demo/odom is the only way to see that.
    RCLCPP_INFO(rclcpp::get_logger("StateTrotting"),
                "trot supervisor: mode=%s cmd=(%.4f,%.4f,%.4f) tilt=%.1fdeg "
                "posErrXY=%.4f velErrXY=%.4f contact=[%d %d %d %d] "
                "estPos=(%.3f,%.3f,%.3f) estVel=(%.3f,%.3f) estYaw=%.1fdeg "
                "Mz=%.1f/%.1fNm",
                modeName(mode_), v_cmd_body_(0), v_cmd_body_(1), d_yaw_cmd_,
                tilt_ * 180.0 / M_PI,
                pos_error_.head(2).norm(), vel_error_.head(2).norm(),
                wave_generator_->contact_(0), wave_generator_->contact_(1),
                wave_generator_->contact_(2), wave_generator_->contact_(3),
                pos_body_(0), pos_body_(1), pos_body_(2),
                vel_body_(0), vel_body_(1),
                estimator_->getYaw() * 180.0 / M_PI,
                balance_ctrl_->getWrenchDemand()(5),
                balance_ctrl_->getWrenchAchieved()(5));
}
