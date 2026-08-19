//
// Created by biao on 24-9-18.
//

#include "unitree_guide_controller/gait/GaitGenerator.h"

#include <utility>
#include <unitree_guide_controller/control/CtrlComponent.h>
#include <unitree_guide_controller/control/Estimator.h>
#include <unitree_guide_controller/gait/WaveGenerator.h>

GaitGenerator::GaitGenerator(CtrlComponent &ctrl_component)
    : wave_generator_(ctrl_component.wave_generator_),
      estimator_(ctrl_component.estimator_),
      feet_end_calc_(ctrl_component) {
    first_run_ = true;
}

void GaitGenerator::setGait(Vec2 vxy_goal_global, const double d_yaw_goal, const double gait_height) {
    vxy_goal_ = std::move(vxy_goal_global);
    d_yaw_goal_ = d_yaw_goal;
    gait_height_ = gait_height;
}

void GaitGenerator::generate(Vec34 &feet_pos, Vec34 &feet_vel) {
    if (first_run_) {
        start_p_ = estimator_->getFeetPos();
        first_run_ = false;
    }

    for (int i = 0; i < 4; i++) {
        if (wave_generator_->contact_(i) == 1) {
            // Latch where the foot is planted, so the stance target is the
            // ground the leg is standing on.
            //
            // `phase < 0.5` alone is not enough once the gait stops:
            // WaveGenerator pins the phase at exactly 0.5 in STANCE_ALL, so
            // the condition is never true again and the two legs that were
            // mid-swing when the command went to zero keep the target from
            // their lift-off point for as long as the robot stands.  Measured
            // at 6-8 cm on one diagonal pair, constant, with the joint PD
            // pulling on it the whole time -- enough to rotate a standing
            // robot tens of degrees and, twice, to tip it over.
            //
            // Latching on touchdown covers both cases: during the gait it
            // fires at the start of stance, which `phase < 0.5` already did,
            // and after the gait stops it fires once for each leg as it lands.
            const bool just_landed = contact_past_(i) == 0;
            if (wave_generator_->phase_(i) < 0.5 || just_landed) {
                start_p_.col(i) = estimator_->getFootPos(i);
            }
            feet_pos.col(i) = start_p_.col(i);
            feet_vel.col(i).setZero();
        } else {
            // foot not contact, swing
            end_p_.col(i) = feet_end_calc_.calcFootPos(i, vxy_goal_, d_yaw_goal_, wave_generator_->phase_(i));
            feet_pos.col(i) = getFootPos(i);
            feet_vel.col(i) = getFootVel(i);
        }
        contact_past_(i) = wave_generator_->contact_(i);
    }
}

void GaitGenerator::restart() {
    first_run_ = true;
    vxy_goal_.setZero();
    feet_end_calc_.init();
}


Vec3 GaitGenerator::getFootPos(const int i) {
    Vec3 foot_pos;

    foot_pos(0) =
            cycloidXYPosition(start_p_.col(i)(0), end_p_.col(i)(0), wave_generator_->phase_(i));
    foot_pos(1) =
            cycloidXYPosition(start_p_.col(i)(1), end_p_.col(i)(1), wave_generator_->phase_(i));
    foot_pos(2) = cycloidZPosition(start_p_.col(i)(2), gait_height_, wave_generator_->phase_(i));

    return foot_pos;
}

Vec3 GaitGenerator::getFootVel(const int i) {
    Vec3 foot_vel;

    foot_vel(0) =
            cycloidXYVelocity(start_p_.col(i)(0), end_p_.col(i)(0), wave_generator_->phase_(i));
    foot_vel(1) =
            cycloidXYVelocity(start_p_.col(i)(1), end_p_.col(i)(1), wave_generator_->phase_(i));
    foot_vel(2) = cycloidZVelocity(gait_height_, wave_generator_->phase_(i));

    return foot_vel;
}

double GaitGenerator::cycloidXYPosition(const double startXY, const double endXY, const double phase) {
    const double phase_pi = 2 * M_PI * phase;
    return (endXY - startXY) * (phase_pi - sin(phase_pi)) / (2 * M_PI) + startXY;
}

double GaitGenerator::cycloidZPosition(const double startZ, const double height, const double phase) {
    const double phase_pi = 2 * M_PI * phase;
    return height * (1 - cos(phase_pi)) / 2 + startZ;
}

double GaitGenerator::cycloidXYVelocity(const double startXY, const double endXY, const double phase) const {
    const double phase_pi = 2 * M_PI * phase;
    return (endXY - startXY) * (1 - cos(phase_pi)) / wave_generator_->get_t_swing();
}

double GaitGenerator::cycloidZVelocity(const double height, const double phase) const {
    const double phase_pi = 2 * M_PI * phase;
    return height * M_PI * sin(phase_pi) / wave_generator_->get_t_swing();
}
