//
// Created by tlab-uav on 25-2-27.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <unitree_guide_controller/gait/WaveGenerator.h>

#include "BalanceCtrl.h"
#include "Estimator.h"
#include "GaitParams.h"

struct CtrlComponent {
    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;
    std::shared_ptr<BalanceCtrl> balance_ctrl_;
    std::shared_ptr<WaveGenerator> wave_generator_;

    /**
     * Filled by UnitreeGuideController::on_init(), before anything that reads
     * it is constructed: WaveGenerator and BalanceCtrl in on_configure(),
     * StateTrotting and FeetEndCalc in on_activate().
     */
    GaitParams gait_params_;

    CtrlComponent() = default;
};
#endif //CTRLCOMPONENT_H
