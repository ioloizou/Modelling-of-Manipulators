#include <ecn_manip/robot_kr16.h>
#include <ecn_manip/trig_solvers.h>

// Model of Kuka KR16 robot

// Any end-effector to wrist constant transform
void ecn::RobotKr16::init_wMe()
{
    // Generated end-effector code
       wMe[0][0] = 1.;
       wMe[0][1] = 0;
       wMe[0][2] = 0;
       wMe[0][3] = 0;
       wMe[1][0] = 0;
       wMe[1][1] = -1.;
       wMe[1][2] = 0;
       wMe[1][3] = 0;
       wMe[2][0] = 0;
       wMe[2][1] = 0;
       wMe[2][2] = -1.;
       wMe[2][3] = -0.158000000000000;
       wMe[3][0] = 0;
       wMe[3][1] = 0;
       wMe[3][2] = 0;
       wMe[3][3] = 1.;
       // End of end-effector code


}

// Direct Geometry fixed to wrist frame
vpHomogeneousMatrix ecn::RobotKr16::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;
    // Generated pose code
      const auto c1{cos(q[0])};
      const auto c2{cos(q[1])};
      const auto c4{cos(q[3])};
      const auto c5{cos(q[4])};
      const auto c6{cos(q[5])};
      const auto c23{cos(q[1]+q[2])};
      const auto s1{sin(q[0])};
      const auto s2{sin(q[1])};
      const auto s4{sin(q[3])};
      const auto s5{sin(q[4])};
      const auto s6{sin(q[5])};
      const auto s23{sin(q[1]+q[2])};
      M[0][0] = (-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*c6 - (s1*c4 - s4*s23*c1)*s6;
      M[0][1] = -(-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*s6 - (s1*c4 - s4*s23*c1)*c6;
      M[0][2] = (s1*s4 + s23*c1*c4)*s5 - c1*c5*c23;
      M[0][3] = (-0.035*s23 + 0.68*c2 + 0.67*c23 + 0.26)*c1;
      M[1][0] = ((s1*s23*c4 - s4*c1)*c5 + s1*s5*c23)*c6 - (s1*s4*s23 + c1*c4)*s6;
      M[1][1] = -((s1*s23*c4 - s4*c1)*c5 + s1*s5*c23)*s6 - (s1*s4*s23 + c1*c4)*c6;
      M[1][2] = -(s1*s23*c4 - s4*c1)*s5 + s1*c5*c23;
      M[1][3] = (0.035*s23 - 0.68*c2 - 0.67*c23 - 0.26)*s1;
      M[2][0] = (s5*s23 - c4*c5*c23)*c6 + s4*s6*c23;
      M[2][1] = -(s5*s23 - c4*c5*c23)*s6 + s4*c6*c23;
      M[2][2] = s5*c4*c23 + s23*c5;
      M[2][3] = -0.68*s2 - 0.67*s23 - 0.035*c23 + 0.675;
      M[3][0] = 0;
      M[3][1] = 0;
      M[3][2] = 0;
      M[3][3] = 1.;
      // End of pose code



    return M;
}


// Inverse Geometry
vpColVector ecn::RobotKr16::inverseGeometry(const vpHomogeneousMatrix &fMe_des, const vpColVector &q0) const
{
    // build corresponding oMw and explode into 12 elements
    const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz,tx,ty,tz] = explodeMatrix(fMe_des);


    // TODO add candidates


            // TODO add candidates
                    // for this robot, we are going to divide in translation part and rotation part

                    //Translation Part
                    //Notice that the  coeficient of second equation is equals to the coeficient of first equation, so, if, we divide then, we get: tan(q1) = -ty/tx

                    auto q1{atan2(-ty,tx)};
                    const auto c1{cos(q1)};
                    const auto s1{sin(q1)};

                   for(auto [q2, q23]: solveType7 (-0.68, 0, (tx/c1)-0.26, -tz+0.675, 0.67, -0.035))

                     {
                       const auto c23{cos(q23)};
                       const auto s23{sin(q23)};

                       const auto c2{cos(q2)};
                       const auto s2{sin(q2)};

                     auto q3 = q23 -q2;

                    //Rotation Part
                     vpRotationMatrix R03;
                     vpRotationMatrix R36;

                     //Rotation 0R3 from root frame to frame 3:
                     R03[0][0] = s23*c1;
                     R03[0][1] = -c1*c23;
                     R03[0][2] = -s1;
                     R03[1][0] = -s1*s23;
                     R03[1][1] = s1*c23;
                     R03[1][2] = -c1;
                     R03[2][0] = c23;
                     R03[2][1] = s23;
                     R03[2][2] = 0;

                     R36 = R03.t() * (fMe_des.getRotationMatrix());



                    for(auto q5: solveType2 (0, -1, R36[1][2]))
                    {

                      if (q5 != isNull(q5)){

                          auto s5 = sin(q5);
                        for(auto q6: solveType3(0, -s5, R36[1][0], -s5, 0, R36[1][1]))
                        {

                            for(auto q4: solveType3 (0, -s5, R36[0][2], s5, 0, R36[2][2]))
                            {

                                addCandidate({q1,q2,q3,q4,q5,q6+M_PI});
                            }
                        }
                      }
                      else{
                          for(auto q46: solveType2(0, 1, R36[0][0])){
                             const auto c46{cos(q46)};
                             const auto s46{sin(q46)};

                              auto q6 = 0.5*(q46-q0[3]+q0[5]);
                              auto q4 = q46 - q6;

                              const auto c6{cos(q6)};
                              const auto s6{sin(q6)};
                                cout <<q6<<endl;

                               const auto c4{cos(q4)};
                               const auto s4{sin(q4)};
                                cout <<q4<<endl;

                               addCandidate({q1,q2,q3,q4,q5,q6});




                      }

                    }

                  }
                }



    return bestCandidate(q0);
}


vpMatrix ecn::RobotKr16::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);


    // Generated Jacobian code
        const auto c1{cos(q[0])};
        const auto c2{cos(q[1])};
        const auto c4{cos(q[3])};
        const auto c5{cos(q[4])};
        const auto c23{cos(q[1]+q[2])};
        const auto s1{sin(q[0])};
        const auto s2{sin(q[1])};
        const auto s4{sin(q[3])};
        const auto s5{sin(q[4])};
        const auto s23{sin(q[1]+q[2])};
        J[0][0] = (0.035*s23 - 0.68*c2 - 0.67*c23 - 0.26)*s1;
        J[0][1] = -(0.68*s2 + 0.67*s23 + 0.035*c23)*c1;
        J[0][2] = -(0.67*s23 + 0.035*c23)*c1;
        J[0][3] = 0;
        J[0][4] = 0;
        J[0][5] = 0;
        J[1][0] = -(-0.035*s23 + 0.68*c2 + 0.67*c23 + 0.26)*c1;
        J[1][1] = (0.68*s2 + 0.67*s23 + 0.035*c23)*s1;
        J[1][2] = (0.67*s23 + 0.035*c23)*s1;
        J[1][3] = 0;
        J[1][4] = 0;
        J[1][5] = 0;
        J[2][0] = 0;
        J[2][1] = 0.035*s23 - 0.68*c2 - 0.67*c23;
        J[2][2] = 0.035*s23 - 0.67*c23;
        J[2][3] = 0;
        J[2][4] = 0;
        J[2][5] = 0;
        J[3][0] = 0;
        J[3][1] = s1;
        J[3][2] = s1;
        J[3][3] = -c1*c23;
        J[3][4] = s1*c4 - s4*s23*c1;
        J[3][5] = (s1*s4 + s23*c1*c4)*s5 - c1*c5*c23;
        J[4][0] = 0;
        J[4][1] = c1;
        J[4][2] = c1;
        J[4][3] = s1*c23;
        J[4][4] = s1*s4*s23 + c1*c4;
        J[4][5] = -(s1*s23*c4 - s4*c1)*s5 + s1*c5*c23;
        J[5][0] = -1.;
        J[5][1] = 0;
        J[5][2] = 0;
        J[5][3] = s23;
        J[5][4] = -s4*c23;
        J[5][5] = s5*c4*c23 + s23*c5;
      // End of Jacobian code



    return J;
}
