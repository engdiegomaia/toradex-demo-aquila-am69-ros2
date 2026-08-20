//
// Gait tuning surface for the Go2 port.
//

#ifndef GAITPARAMS_H
#define GAITPARAMS_H

#include <unitree_guide_controller/common/mathTypes.h>

/**
 * Every number the trot is tuned by, in one place, declared as ROS parameters
 * by UnitreeGuideController::on_init().
 *
 * The reason this struct exists is arithmetic, not taste: every gait experiment
 * recorded in docs/results/ml35-f4-parcial.md cost a container rebuild, because
 * the value being swept was a literal in a constructor.  Four of those
 * experiments were reverted, which means the sweep has to be re-run whenever a
 * neighbouring value changes, and a rebuild per point makes that a day instead
 * of an afternoon.
 *
 * The defaults below are the values the controller shipped with at the moment
 * of extraction, so a run with no YAML at all reproduces the recorded baseline.
 * They are duplicated in go2_description/config/gazebo.yaml on purpose: the
 * YAML is what an experiment edits, and a value that only exists here cannot be
 * swept without a rebuild -- which is the thing this struct removes.  Keep the
 * two in step; the F4 gate is re-run against the YAML.
 *
 * The measured justification for each non-upstream value stays next to the code
 * that consumes it, not here, so that reading the control law still explains
 * itself: k_yaw in FeetEndCalc.cpp, the yaw clamp and kp_w in
 * StateTrotting::calcTau, weight_moment in BalanceCtrl.
 */
struct GaitParams {
    /* WaveGenerator: the contact schedule. */
    double gait_period{0.45}; //!< s, one full cycle; t_stance = t_swing = 0.225
    double gait_stance_ratio{0.5}; //!< fraction of the cycle a leg spends in stance
    double gait_height{0.08}; //!< m, swing apex above the stance plane

    /* FeetEndCalc: where the swing foot is put down.
     *
     * k_x/k_y are the Raibert velocity-feedback gains and k_yaw the heading
     * one.  0.005 is upstream's A1 value for all three; k_yaw was raised to
     * 0.15 after measuring that it is the only actuator with authority over
     * heading on this robot.  See FeetEndCalc.cpp. */
    double k_x{0.005};
    double k_y{0.005};
    double k_yaw{0.15};

    /* StateTrotting: body pose control feeding the QP. */
    Vec3 kp_p{Vec3(70, 70, 70)};
    Vec3 kd_p{Vec3(10, 10, 10)};
    double kp_w{780};
    Vec3 kd_w{Vec3(70, 70, 70)};
    Vec3 kp_swing{Vec3(400, 400, 400)};
    Vec3 kd_swing{Vec3(10, 10, 10)};

    /* StateTrotting: command envelope, and how far the integrated body
     * reference may run ahead of the body.  See calcCmd(). */
    Vec2 v_x_limit{Vec2(-0.4, 0.4)};
    Vec2 v_y_limit{Vec2(-0.3, 0.3)};
    Vec2 w_yaw_limit{Vec2(-0.5, 0.5)};
    double reference_band{0.01}; //!< m

    /* StateTrotting::calcTau: clamps on the demand handed to the QP.
     * ang_acc_limit_yaw doubles as the threshold of the yaw saturation
     * statistic, so the duty-cycle number keeps meaning "at the rail". */
    double acc_limit_xy{3.0}; //!< m/s^2
    double acc_limit_z{5.0}; //!< m/s^2
    double ang_acc_limit_roll_pitch{40.0}; //!< rad/s^2
    double ang_acc_limit_yaw{10.0}; //!< rad/s^2, 5.3 N.m on this robot

    /* BalanceCtrl: QP residual weights and friction cone.
     *
     * weight_force/weight_moment are the diagonal of S_.  The ratio is what
     * makes an unreachable yaw moment damaging rather than merely ignored: the
     * solver trades force distribution to chase it.  friction_ratio is the
     * cone the QP assumes, deliberately below the simulated foot friction
     * (mu = 0.6 in go2_description/xacro/leg.xacro) because it is the number
     * that has to survive on real ground. */
    Vec3 weight_force{Vec3(20, 20, 50)};
    Vec3 weight_moment{Vec3(450, 450, 450)};
    double friction_ratio{0.4};

    /* StateTrotting: posture settle while standing (MotionMode::HOLD).
     *
     * Both default to the current behaviour -- hold_weight_moment_yaw equal to
     * weight_moment(2), and a settle rate of zero -- so a run with no YAML
     * reproduces the recorded baseline, which is the invariant the rest of this
     * struct is built on.  The measured justification is next to the code that
     * consumes them, in StateTrotting::applyHoldYawWeight and
     * StateTrotting::settleHoldPosture. */
    double hold_weight_moment_yaw{450.0}; //!< QP yaw-moment weight while standing
    double hold_settle_rate{0.0}; //!< m/s, rate the parked reference walks to the support centroid
};

#endif //GAITPARAMS_H
