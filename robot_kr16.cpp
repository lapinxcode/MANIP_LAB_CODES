#include <ecn_manip/robot_kr16.h>
#include <ecn_manip/trig_solvers.h>

// Model of Kuka KR16 robot

// Any end-effector to wrist constant transform
void ecn::RobotKr16::init_wMe()
{
    wMe[0][0] = -1.;
    wMe[0][1] = 0;
    wMe[0][2] = 0;
    wMe[0][3] = 0;
    wMe[1][0] = 0;
    wMe[1][1] = 1.;
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
      // numerical values to initialize

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
  M[0][0] = ((s1*s4 + s23*c1*c4)*c5 + s5*c1*c23)*c6 + (s1*c4 - s4*s23*c1)*s6;
  M[0][1] = -((s1*s4 + s23*c1*c4)*c5 + s5*c1*c23)*s6 + (s1*c4 - s4*s23*c1)*c6;
  M[0][2] = (s1*s4 + s23*c1*c4)*s5 - c1*c5*c23;
  M[0][3] = (-0.035*s23 + 0.68*c2 + 0.67*c23 + 0.26)*c1;
  M[1][0] = ((-s1*s23*c4 + s4*c1)*c5 - s1*s5*c23)*c6 + (s1*s4*s23 + c1*c4)*s6;
  M[1][1] = -((-s1*s23*c4 + s4*c1)*c5 - s1*s5*c23)*s6 + (s1*s4*s23 + c1*c4)*c6;
  M[1][2] = (-s1*s23*c4 + s4*c1)*s5 + s1*c5*c23;
  M[1][3] = (0.035*s23 - 0.68*c2 - 0.67*c23 - 0.26)*s1;
  M[2][0] = (-s5*s23 + c4*c5*c23)*c6 - s4*s6*c23;
  M[2][1] = -(-s5*s23 + c4*c5*c23)*s6 - s4*c6*c23;
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
vpColVector ecn::RobotKr16::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
  // desired wrist position
  const auto [tx,ty,tz] = explodeTranslation(Md);
  vpRotationMatrix R03;

  auto q1 = atan2(-ty,tx);
  //const auto c1{cos(q1)};
  auto s1{sin(q1)};

  // first solve position for (q1,q2,q3).
  for(auto [q2,q23] : solveType7(-0.67,0.035,(-ty/s1)-0.26,-tz+0.675,0.68,0)){

      const auto q3 = q23 - q2;
      //const auto c2{cos(q2)};
      //const auto s2{sin(q2)};
      //const auto c3{cos(q3)};
      //const auto s3{sin(q3)};

      const auto c1{cos(q1)};
      const auto c23{cos(q23)};
      const auto s1{sin(q1)};
      const auto s23{sin(q23)};

      // then (inside the last for block) build R36 and solve it for (q4,q5,q6)

      R03[0][0] = -s23*c1;
      R03[0][1] = -c1*c23;
      R03[0][2] = s1;
      R03[1][0] = s1*s23;
      R03[1][1] = s1*c23;
      R03[1][2] = c1;
      R03[2][0] = -c23;
      R03[2][1] = s23;
      R03[2][2] = 0;

      // Elements of 3R6
      const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz] = explodeWristMatrix(Md, R03);

      for(auto q5 : solveType2(0,1,zy)){
          //const auto c5{cos(q5)};
          const auto s5{sin(q5)};
          for(auto q4 : solveType3(0,-s5,zx,s5,0,zz)){
              //const auto c4{cos(q4)};
              //const auto s4{sin(q4)};
              for(auto q6 : solveType3(0,-s5,xy,s5,0,yy)){
                addCandidate({q1,q2,q23-q2,q4,q5,q6});
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
  // numerical values to initialize

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
  J[4][5] = (-s1*s23*c4 + s4*c1)*s5 + s1*c5*c23;
  J[5][0] = -1.;
  J[5][1] = 0;
  J[5][2] = 0;
  J[5][3] = s23;
  J[5][4] = -s4*c23;
  J[5][5] = s5*c4*c23 + s23*c5;
  // End of Jacobian code

  return J;
}
