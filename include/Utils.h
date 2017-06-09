//
// Created by rochelle on 17-5-23.
//

#ifndef DIRECTSPARSEVO_UTILS_H
#define DIRECTSPARSEVO_UTILS_H

#include "Common.h"

#define SQR(X) (X)*(X)

struct Translation
{
    double x, y, z;
};

struct Quaternion
{
    double q0, q1, q2, q3;
};

Eigen::Matrix<double,3,3> Qua2Mat(const Quaternion& quat)
{
    Eigen::Matrix<double,3,3> R;

    R(0, 0) = SQR(quat.q0) + SQR(quat.q1) - SQR(quat.q2) - SQR(quat.q3);
    R(0, 1) = 2 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
    R(0, 2) = 2 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);

    R(1, 0) = 2 * (quat.q1 * quat.q2 - quat.q0 * quat.q3);
    R(1, 1) = SQR(quat.q0) - SQR(quat.q1) + SQR(quat.q2) - SQR(quat.q3);
    R(1, 2) = 2 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);

    R(2, 0) = 2 * (quat.q1 * quat.q3 + quat.q0 * quat.q2);
    R(2, 1) = 2 * (quat.q2 * quat.q3 - quat.q0 * quat.q1);
    R(2, 2) = SQR(quat.q0) - SQR(quat.q1) - SQR(quat.q2) + SQR(quat.q3);

    return R;
};

// same R may have several quaternion representation
Quaternion Mat2Qua(const Eigen::Matrix<double,3,3> R)
{
    Quaternion quat;

    double trace = R.trace();

    quat.q0 = sqrt(R.trace() + 1) / 2;
    double _4q0 = 4*quat.q0;
    quat.q1 = (R(1, 2) - R(2, 1)) / _4q0;
    quat.q2 = (R(2, 0) - R(0, 2)) / _4q0;
    quat.q3 = (R(0, 1) - R(1, 0)) / _4q0;

    return quat;
}

Eigen::Matrix<double,4,4> TransQua2Mat(const Translation& trans,
                                       const Quaternion& quat)
{
    Eigen::Matrix<double,4,4> transform;

    transform.block<3, 3>(0, 0) = Qua2Mat(quat);
    transform.block<3, 1>(0, 3) << trans.x, trans.y, trans.z;
    transform.block<1, 4>(3, 0) << 0.0, 0.0, 0.0, 1.0;

    return transform;
}

#endif //DIRECTSPARSEVO_UTILS_H
