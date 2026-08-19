//
// Created by biao on 24-9-18.
//

#include "unitree_guide_controller/gait/FeetEndCalc.h"

#include <unitree_guide_controller/control/CtrlComponent.h>
#include <unitree_guide_controller/control/Estimator.h>

FeetEndCalc::FeetEndCalc(CtrlComponent &ctrl_component)
    : ctrl_component_(ctrl_component),
      robot_model_(ctrl_component.robot_model_),
      estimator_(ctrl_component.estimator_) {
    // Raibert velocity-feedback gains.  Upstream ships 0.005 for all three,
    // which is the A1's value and is 4% of what the neutral term it opposes
    // carries; the yaw one was already raised on measured grounds (below).
    // They are parameters so a sweep is a YAML edit, not a rebuild.
    k_x_ = ctrl_component.gait_params_.k_x;
    k_y_ = ctrl_component.gait_params_.k_y;

    // Heading correction gain, upstream 0.005.  calcFootPos places each foot at
    //   angle = yaw + feet_init_angle_(i) + next_yaw
    //   next_yaw = d_yaw*(1-phase)*t_swing + d_yaw*t_stance/2 + k_yaw_*(0 - d_yaw)
    // The first two terms rotate the landing point around the body by the yaw
    // rate the body already has -- the neutral, rate-preserving placement.  At
    // touchdown (phase -> 1) that coefficient is t_stance/2 = 0.1125 s, so at
    // 1 rad/s the whole support pattern is laid down 6.4 degrees rotated, and
    // the swinging diagonal pair sweeps inward: the legs visibly cross toward
    // the body centre, which yaws the body further, which rotates the next
    // placement more.  k_yaw_ = 0.005 is the only term opposing that, at 4% of
    // what it has to cancel.
    //
    // 0.15 cancels the 0.1125 carried at touchdown and leaves a corrective
    // margin, so the feet land in a pattern that resists the spin instead of
    // following it.  This is the only actuator that can hold heading on this
    // robot: the balance QP tops out near 5.3 N.m of yaw moment, which is not
    // enough to regulate it (measured -- see StateTrotting's gain comment).
    k_yaw_ = ctrl_component.gait_params_.k_yaw;
}

void FeetEndCalc::init() {
    t_stance_ = ctrl_component_.wave_generator_->get_t_stance();
    t_swing_ = ctrl_component_.wave_generator_->get_t_swing();

    Vec34 feet_pos_body = estimator_->getFeetPos2Body();
    // Vec34 feet_pos_body = robot_model_.feet_pos_normal_stand_;
    for (int i(0); i < 4; ++i) {
        feet_radius_(i) =
                sqrt(pow(feet_pos_body(0, i), 2) + pow(feet_pos_body(1, i), 2));
        feet_init_angle_(i) = atan2(feet_pos_body(1, i), feet_pos_body(0, i));
    }
}

Vec3 FeetEndCalc::calcFootPos(const int index, Vec2 vxy_goal_global, const double d_yaw_global, const double phase) {
    Vec3 body_vel_global = estimator_->getVelocity();
    Vec3 next_step;

    next_step(0) = body_vel_global(0) * (1 - phase) * t_swing_ +
                   body_vel_global(0) * t_stance_ / 2 +
                   k_x_ * (body_vel_global(0) - vxy_goal_global(0));
    next_step(1) = body_vel_global(1) * (1 - phase) * t_swing_ +
                   body_vel_global(1) * t_stance_ / 2 +
                   k_y_ * (body_vel_global(1) - vxy_goal_global(1));
    next_step(2) = 0;

    const double yaw = estimator_->getYaw();
    const double d_yaw = estimator_->getDYaw();
    const double next_yaw = d_yaw * (1 - phase) * t_swing_ + d_yaw * t_stance_ / 2 +
                            k_yaw_ * (d_yaw_global - d_yaw);

    next_step(0) +=
            feet_radius_(index) * cos(yaw + feet_init_angle_(index) + next_yaw);
    next_step(1) +=
            feet_radius_(index) * sin(yaw + feet_init_angle_(index) + next_yaw);

    Vec3 foot_pos = estimator_->getPosition() + next_step;
    foot_pos(2) = 0.0;

    return foot_pos;
}
