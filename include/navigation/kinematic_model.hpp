#ifndef KINEMATIC_MODEL_HPP
#define KINEMATIC_MODEL_HPP

class KinematicModel{
private:
    double L1, L2, r;

public:
    KinematicModel(
        double L1,
        double L2,
        double r
    ) :
    L1(L1), L2(L2), r(r)
    {}

    inline void computeRobotVelocityFromWheelVelocity(
        double omega1,
        double omega2,
        double omega3,
        double &v,
        double &omega
    )
    {
        v = (r / 2.0) * (omega1 + omega2);
        omega = omega * L2 / r;
    }
    inline void computeWheelVelocityFromRobotVelocity(
        double v,
        double omega,
        double &omega1,
        double &omega2,
        double &omega3
    )
    {
        omega1 = (1.0 / r) * (v - L1 * omega);
        omega2 = (1.0 / r) * (v + L1 * omega);
        omega3 = (1.0 / r) * omega * L2;
    }
};


#endif // KINEMATIC_MODEL_HPP