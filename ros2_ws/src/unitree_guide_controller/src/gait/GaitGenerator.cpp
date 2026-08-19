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
            //
            // Latching once is still not enough once the gait *stays* stopped.
            // The target is a point in the estimator frame, and this robot's
            // estimator has no absolute XY measurement: position is observable
            // only while the contacting feet are genuinely still.  Measured
            // over a 90 s stop, the estimate drifted 0.18 m away from ground
            // truth, and the latched target carried that whole difference into
            // the joint PD as a body-relative foot offset (`pos_feet_target`
            // in StateTrotting is `goal - pos_body_`, so a drifting body moves
            // the target under the foot).  The offset makes the legs push, the
            // push makes the feet slip, and the slip feeds the drift back.
            // After a straight walk the loop settled at 0.18 m and the robot
            // held; after a walk with yaw it did not settle -- the error blew
            // through 0.2 m at 15 s and the robot rolled over at 18.3 s.
            // Restarting the trot from an already-drifted stance fell in 2.0 s.
            //
            // In STANCE_ALL all four feet are down and no swing runs until the
            // gait restarts, so re-latching every tick costs nothing and keeps
            // the stance target on the foot instead of on a stale point.  The
            // loop then never closes.  Station keeping is unaffected: it comes
            // from the QP regulating the body against `pcd_`, not from this PD.
            const bool just_landed = contact_past_(i) == 0;
            const bool gait_stopped = wave_generator_->status_ == WaveStatus::STANCE_ALL;
            if (gait_stopped || wave_generator_->phase_(i) < 0.5 || just_landed) {
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
