//
// Created by tlab-uav on 24-9-18.
//

#ifndef STATETROTTING_H
#define STATETROTTING_H
#include <unitree_guide_controller/control/BalanceCtrl.h>
#include <unitree_guide_controller/control/GaitParams.h>
#include <unitree_guide_controller/gait/GaitGenerator.h>
#include "controller_common/FSM/FSMState.h"

/**
 * What the trot supervisor decided to do on this control tick.
 *
 * Upstream TROTTING has no such concept: it always integrates the velocity
 * command into a body reference, and a threshold on the resulting tracking
 * error decides whether the feet leave the ground.  Those two decisions are
 * independent, so the state can (and on this robot did) push the body sideways
 * with all four feet planted, and keep pushing after the command went back to
 * zero.  Splitting the behaviour into three explicit modes makes both the gait
 * activation and the stop condition observable in the diagnostics line.
 */
enum class MotionMode {
    HOLD, //!< four feet down, body parked on the captured reference
    WALK, //!< gait enabled, body reference tracks the velocity command
    RECOVER, //!< tilted out of the safe band: cancel the command, level the body
};

class StateTrotting final : public FSMState {
public:
    explicit StateTrotting(CtrlInterfaces &ctrl_interfaces,
                           CtrlComponent &ctrl_component);

    void enter() override;

    void run(const rclcpp::Time &time,
             const rclcpp::Duration &period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    void getUserCmd();

    void calcCmd();

    /**
    * Calculate the torque command
    */
    void calcTau();

    /**
    * Calculate the joint space velocity and acceleration
    */
    void calcQQd();

    /**
    * Calculate the PD gain for the joints
    */
    void calcGain() const;

    /**
     * Update tilt_, mode_ and walking_ from the current command and attitude.
     * Runs before calcCmd() so that a cancelled command never reaches the
     * body reference in the first place.
     */
    void updateMotionMode();

    /**
     * Latch the walk decision with hysteresis, from commanded motion only.
     * @return whether the gait should be running this tick
     */
    bool updateWalkIntent();

    /**
     * Drop the velocity and yaw-rate command, filter state included.
     */
    void cancelCommand();

    /**
     * Park the horizontal position and yaw reference on the current pose, so
     * that leaving WALK leaves no residual reference for BalanceCtrl to chase.
     */
    void captureBodyReference();

    /** Schedule the QP yaw-moment weight by motion mode. */
    void applyHoldYawWeight();

    /** Walk the parked body reference toward the centroid of the stance feet. */
    void settleHoldPosture();

    /**
     * One line per second: mode, command, tilt, tracking error and contacts.
     */
    void logDiagnostics();

    std::shared_ptr<Estimator> &estimator_;
    std::shared_ptr<QuadrupedRobot> &robot_model_;
    std::shared_ptr<BalanceCtrl> &balance_ctrl_;
    std::shared_ptr<WaveGenerator> &wave_generator_;

    GaitGenerator gait_generator_;

    /**
     * Tuning, owned by CtrlComponent and filled from ROS parameters before this
     * state is constructed.  Held by reference rather than copied so that the
     * clamps read at every tick and the gains read once in the constructor
     * cannot drift apart.
     */
    const GaitParams &params_;

    // Robot State
    Vec3 pos_body_, vel_body_;
    RotMat B2G_RotMat, G2B_RotMat;

    // Robot command
    Vec3 pcd_;
    Vec3 vel_target_, v_cmd_body_;
    double dt_;
    double yaw_cmd_{}, d_yaw_cmd_{}, d_yaw_cmd_past_{};
    Vec3 w_cmd_global_;
    Vec34 pos_feet_global_goal_, vel_feet_global_goal_;
    RotMat Rd;

    // Motion supervisor
    MotionMode mode_{MotionMode::HOLD};
    bool walking_{false};
    bool hold_captured_{false};
    //!< Last yaw-moment weight pushed into the QP; negative means "not yet set".
    double yaw_weight_applied_{-1.0};
    double tilt_{};
    double settled_s_{};
    double entry_s_{};
    int diag_ticks_{};

    // Yaw-axis relay statistics.  The control loop runs at 500 Hz and the
    // diagnostic line at 4 Hz, so an instantaneous sample of a bang-bang axis
    // aliases into noise.  What characterises a relay is the size of its
    // demand and its duty cycle at the rail, accumulated over the window.
    double yaw_err_{};
    double yaw_err_peak_{};
    double d_wz_peak_{};
    double d_wz_sum_{};
    int yaw_sat_ticks_{};
    int yaw_win_ticks_{};

    // Control Parameters
    double gait_height_;
    Vec3 pos_error_, vel_error_;
    Mat3 Kpp, Kdp, Kd_w_;
    double kp_w_;
    Mat3 Kp_swing_, Kd_swing_;
    Vec2 v_x_limit_, v_y_limit_, w_yaw_limit_;
};


#endif //STATETROTTING_H
