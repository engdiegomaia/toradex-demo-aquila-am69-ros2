//
// Created by tlab-uav on 24-9-16.
//

#include "unitree_guide_controller/control/BalanceCtrl.h"

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/robot/QuadrupedRobot.h>

#include "quadProgpp/QuadProg++.hh"

BalanceCtrl::BalanceCtrl(const std::shared_ptr<QuadrupedRobot> &robot, const GaitParams &params) {
    mass_ = robot->mass_;

    alpha_ = 0.001;
    beta_ = 0.1;
    g_ << 0, 0, -9.81;
    friction_ratio_ = params.friction_ratio;
    friction_mat_ << 1, 0, friction_ratio_, -1, 0, friction_ratio_, 0, 1, friction_ratio_, 0, -1,
            friction_ratio_, 0, 0, 1;

    pcb_ = Vec3(0.0, 0.0, 0.0);
    // Whole-robot inertia about the COM, in the body frame -- see calVectorBd,
    // which uses it as R * Ib_ * R^T * dWbd.  The upstream value
    // (0.0792, 0.2085, 0.2265) is the A1's and stayed here through the Go2
    // port, under-stating this robot by a factor of 2.3 on every axis: the QP
    // then asks for 43% of the moment needed to arrest a tilt, which is why
    // the trot lost attitude and fell while HOLD stayed level.
    //
    // Computed from go2_description at the stand pose (hip 0, thigh 0.8,
    // calf -1.5), summing every link with the parallel-axis theorem:
    // 15.098 kg, COM (-0.0016, 0.0, -0.0231) m, diag (0.1817, 0.4899, 0.5262).
    // Off-diagonal terms are below 4% of the diagonal and are dropped, as
    // upstream does.
    Ib_ = Vec3(0.1817, 0.4899, 0.5262).asDiagonal();

    Vec6 s;
    Vec12 w, u;
    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    // Residual weights of the QP, force first then moment.  The ratio between
    // the two halves is what makes an unreachable moment harmful instead of
    // merely unmet: with 450 against 20/20/50 the solver spends the force
    // budget chasing it.  Measured on the yaw axis -- see the clamp note in
    // StateTrotting::calcTau and docs/results/ml35-f4-parcial.md.
    s.head(3) = params.weight_force;
    s.tail(3) = params.weight_moment;

    S_ = s.asDiagonal();
    W_ = w.asDiagonal();
    U_ = u.asDiagonal();

    F_prev_.setZero();
}

void BalanceCtrl::setYawMomentWeight(const double weight) {
    // S_ is otherwise built once, in the constructor, from params.weight_moment.
    // Only the yaw entry moves: roll and pitch have authority this axis does
    // not, so there is no measured reason to reweight them.
    S_(5, 5) = weight;
}

Vec34 BalanceCtrl::calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rot_matrix,
                        const Vec34 &feet_pos_2_body, const VecInt4 &contact) {
    calMatrixA(feet_pos_2_body, rot_matrix);
    calVectorBd(ddPcd, dWbd, rot_matrix);
    calConstraints(contact);

    G_ = A_.transpose() * S_ * A_ + alpha_ * W_ + beta_ * U_;
    g0T_ = -bd_.transpose() * S_ * A_ - beta_ * F_prev_.transpose() * U_;

    solveQP();

    wrench_achieved_ = A_ * F_;
    F_prev_ = F_;
    return vec12ToVec34(F_);
}

void BalanceCtrl::calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM) {
    for (int i = 0; i < 4; ++i) {
        A_.block(0, 3 * i, 3, 3) = I3;
        A_.block(3, 3 * i, 3, 3) = skew(Vec3(feet_pos_2_body.col(i)) - rotM * pcb_);
    }
}

void BalanceCtrl::calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM) {
    bd_.head(3) = mass_ * (ddPcd - g_);
    bd_.tail(3) = rotM * Ib_ * rotM.transpose() * dWbd;
}

void BalanceCtrl::calConstraints(const VecInt4 &contact) {
    int contactLegNum = 0;
    for (int i(0); i < 4; ++i) {
        if (contact[i] == 1) {
            contactLegNum += 1;
        }
    }

    CI_.resize(5 * contactLegNum, 12);
    ci0_.resize(5 * contactLegNum);
    CE_.resize(3 * (4 - contactLegNum), 12);
    ce0_.resize(3 * (4 - contactLegNum));

    CI_.setZero();
    ci0_.setZero();
    CE_.setZero();
    ce0_.setZero();

    int ceID = 0;
    int ciID = 0;
    for (int i(0); i < 4; ++i) {
        if (contact[i] == 1) {
            CI_.block(5 * ciID, 3 * i, 5, 3) = friction_mat_;
            ++ciID;
        } else {
            CE_.block(3 * ceID, 3 * i, 3, 3) = I3;
            ++ceID;
        }
    }
}

void BalanceCtrl::solveQP() {
    const long n = F_.size();
    const long m = ce0_.size();
    const long p = ci0_.size();

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = G_(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = CE_.transpose()(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = CI_.transpose()(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        g0[i] = g0T_[i];
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = ce0_[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = ci0_[i];
    }

    solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    for (int i = 0; i < n; ++i) {
        F_[i] = x[i];
    }
}
