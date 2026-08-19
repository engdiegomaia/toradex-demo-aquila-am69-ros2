//
// Created by tlab-uav on 24-9-16.
//

#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include <memory>

#include "unitree_guide_controller/common/mathTypes.h"
class QuadrupedRobot;

class BalanceCtrl {
public:
    explicit BalanceCtrl(const std::shared_ptr<QuadrupedRobot>& robot);

    ~BalanceCtrl() = default;

    /**
     * Calculate the desired feet end force
     * @param ddPcd desired body acceleration
     * @param dWbd desired body angular acceleration
     * @param rot_matrix current body rotation matrix
     * @param feet_pos_2_body feet positions to body under world frame
     * @param contact feet contact
     * @return
     */
    /**
     * Wrench the QP was asked for on the last calF(), as [force(3); moment(3)].
     */
    [[nodiscard]] const Vec6 &getWrenchDemand() const { return bd_; }

    /**
     * Wrench the contact forces actually produce, A_ * F_.  It differs from the
     * demand whenever the friction cone or the unilateral contact constraint
     * binds, and the difference is invisible from outside the QP: the state
     * machine keeps commanding an angular acceleration it never receives.
     * Yaw is the axis at risk, because a trot has two feet down and yaw moment
     * can only come from tangential (friction-limited) force, while roll and
     * pitch can be produced by the much larger normal forces.
     */
    [[nodiscard]] const Vec6 &getWrenchAchieved() const { return wrench_achieved_; }

    Vec34 calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rot_matrix,
               const Vec34 &feet_pos_2_body, const VecInt4 &contact);

private:
    void calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM);

    void calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM);

    void calConstraints(const VecInt4 &contact);

    void solveQP();

    Mat12 G_, W_, U_;
    Mat6 S_;
    Mat3 Ib_;
    Vec6 bd_;
    Vec6 wrench_achieved_;
    Vec3 g_, pcb_;
    Vec12 F_, F_prev_, g0T_;
    double mass_, alpha_, beta_, friction_ratio_;
    Eigen::MatrixXd CE_, CI_;
    Eigen::VectorXd ce0_, ci0_;
    Eigen::Matrix<double, 6, 12> A_;
    Eigen::Matrix<double, 5, 3> friction_mat_;
};


#endif //BALANCECTRL_H
