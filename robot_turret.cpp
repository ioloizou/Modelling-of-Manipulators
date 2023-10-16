#include <ecn_manip/robot_turret.h>
#include <ecn_manip/trig_solvers.h>

// Model of Turret robot

// Any constant transform at base or end-effector
void ecn::RobotTurret::init_wMe()
{
    // Generated end-effector code
       wMe[0][0] = 1.;
       wMe[0][1] = 0;
       wMe[0][2] = 0;
       wMe[0][3] = 0;
       wMe[1][0] = 0;
       wMe[1][1] = 1.;
       wMe[1][2] = 0;
       wMe[1][3] = 0;
       wMe[2][0] = 0;
       wMe[2][1] = 0;
       wMe[2][2] = 1.;
       wMe[2][3] = 0;
       wMe[3][0] = 0;
       wMe[3][1] = 0;
       wMe[3][2] = 0;
       wMe[3][3] = 1.;
       // End of end-effector code
}

// Direct Geometry
vpHomogeneousMatrix ecn::RobotTurret::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;
    // Generated pose code
      const auto c1{cos(q[0])};
      const auto c2{cos(q[1])};
      const auto s1{sin(q[0])};
      const auto s2{sin(q[1])};

      const auto d=0.1; // meters
      const auto b=0.5; // meters

      M[0][0] = c1*c2;
      M[0][1] = -s1;
      M[0][2] = -s2*c1;
      M[0][3] = (-d - q[2])*s2*c1;
      M[1][0] = s1*c2;
      M[1][1] = c1;
      M[1][2] = -s1*s2;
      M[1][3] = (-d - q[2])*s1*s2;
      M[2][0] = s2;
      M[2][1] = 0;
      M[2][2] = c2;
      M[2][3] = b + (d + q[2])*c2;
      M[3][0] = 0;
      M[3][1] = 0;
      M[3][2] = 0;
      M[3][3] = 1.;
      // End of pose code

    return M;
}


// Inverse Geometry
vpColVector ecn::RobotTurret::inverseGeometry(const vpHomogeneousMatrix &fMe_des, const vpColVector &q0) const
{
    // build corresponding oMw and explode into 12 elements
    const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz,tx,ty,tz] = explodeMatrix(fMe_des);

            // TODO add candidates

    const auto d=0.1; // meters
    const auto b=0.5; // meters

    for(auto q1: solveType3(-1, 0, yx, 0, 1, yy))
    {
        for(auto q2: solveType3(1, 0, xz, 0, 1, zz))
        {
            if (isNull(cos(q2))){
                continue;
               // const auto q3 = tz - b - d;
               // addCandidate({q1,q2,q3});
            }

            else{
            // can divide by c3
            const auto c2{cos(q2)};
            const auto q3 = (tz - b) / c2 - d;
            addCandidate({q1,q2,q3});
            }
        }
    }
    return bestCandidate(q0);
}

// Wrist Jacobian
vpMatrix ecn::RobotTurret::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);

    const auto c1{cos(q[0])};
    const auto c2{cos(q[1])};
    const auto s1{sin(q[0])};
    const auto s2{sin(q[1])};

    const auto d=0.1; //meters

    J[0][0] = (d + q[2])*s1*s2;
    J[0][1] = (-d - q[2])*c1*c2;
    J[0][2] = -s2*c1;
    J[1][0] = (-d - q[2])*s2*c1;
    J[1][1] = (-d - q[2])*s1*c2;
    J[1][2] = -s1*s2;
    J[2][0] = 0;
    J[2][1] = -(d + q[2])*s2;
    J[2][2] = c2;
    J[3][0] = 0;
    J[3][1] = s1;
    J[3][2] = 0;
    J[4][0] = 0;
    J[4][1] = -c1;
    J[4][2] = 0;
    J[5][0] = 1.;
    J[5][1] = 0;
    J[5][2] = 0;
    // End of Jacobian code


    return J;
}
