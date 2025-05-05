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
        double &vx,
        double &vy,
        double &omega
    )
    {
        vx = (r / 2.0) * (omega1 + omega2);
        vy = r * omega3;
        omega = (r / (2.0 * L1)) * (-omega1 + omega2);
    }
    inline void computeWheelVelocityFromRobotVelocity(
        double vx,
        double vy,
        double omega,
        double &omega1,
        double &omega2,
        double &omega3
    )
    {
        omega1 = (1.0 / r) * (vx - L1 * omega);
        omega2 = (1.0 / r) * (vx + L1 * omega);
        omega3 = (1.0 / r) * vy;
    }
};


#endif // KINEMATIC_MODEL_HPP