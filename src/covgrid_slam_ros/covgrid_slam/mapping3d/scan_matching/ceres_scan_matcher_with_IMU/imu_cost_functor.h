#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "utility.h"
#include "parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class IMUCostFunctor
{
  public:
    IMUCostFunctor() = delete;
    explicit IMUCostFunctor(IntegrationBase* _pre_integration, Eigen::Vector3d Ps, Eigen::Quaterniond Qs,
            Eigen::Vector3d Vs, Eigen::Vector3d Bas, Eigen::Vector3d Bgs):
            pre_integration(_pre_integration),
            Ps_l_(Ps), Qs_l_(Qs), Vs_l_(Vs), Bas_l_(Bas), Bgs_l_(Bgs)
    {
    }

    bool operator()(const double* const pj, const double* const qj, const double* const vj, const double* const baj, const double* const bgj,
                    double* const residuals) const
    {
        //init param
        Eigen::Vector3d Pj(pj);
        Eigen::Quaterniond Qj(qj);

        Eigen::Vector3d Vj(vj);
        Eigen::Vector3d Baj(baj);
        Eigen::Vector3d Bgj(bgj);

        //compute residuals

//        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
//            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
//        {
//            //if system state Ba(g) is far away from that in pre_integration,
//            // we repropagate to get more accurate estimated alpha, beta, gama .
//            pre_integration->repropagate(Bai, Bgi);
//        }
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        residual = pre_integration->evaluate(Ps_l_, Qs_l_, Vs_l_, Bas_l_, Bgs_l_,
                                             Pj, Qj, Vj, Baj, Bgj);

        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        residual = sqrt_info * residual;/// Lt(from information Matrix)*residuals

        return true;
    }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    //void checkJacobian(double **parameters);
    IntegrationBase* pre_integration;
    Eigen::Vector3d Ps_l_;
    Eigen::Quaterniond Qs_l_;///double* ptr = Qs[j].coeffs().data(); ptr[i]=num; the way to access data inside Quat
    Eigen::Vector3d Vs_l_;
    Eigen::Vector3d Bas_l_;
    Eigen::Vector3d Bgs_l_;

};

