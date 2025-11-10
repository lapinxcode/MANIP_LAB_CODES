#include <ecn_manip/robot_init.h>
#include <cmath>

using namespace std;
using namespace ecn;

int main(int argc, char ** argv)
{
    // initialize robot class and get DOF's
    const auto robot{initRobot(argc, argv, 100)};
    const unsigned n{robot->getDofs()};

    // robot properties - max velocity and acceleration
    const auto vMax{robot->vMax()};
    const auto aMax{robot->aMax()};

    // main variables
    vpColVector q(n);               // joint position
    vpPoseVector p;                 // operational pose
    vpColVector qCommand(n);        // joint position setpoint
    vpColVector vCommand(n);        // joint velocity setpoint

    vpMatrix J;
    vpHomogeneousMatrix M;          // current pose
    vpHomogeneousMatrix M0, Md, Mi; // previous, final and current desired poses
    vpPoseVector pd;                // desired pose
    vpColVector v;                  // desired operational velocity

    // TODO declare other variables if needed
    vpColVector q0(n), qf(n);        // joint position setpoint for initial and final poses
    double t0, tf;

    // main control loop
    while(robot->ok())
    {

        // current time [s]
        const auto t{robot->time()};

        // update desired pose if has changed
        if(robot->newRef())
        {
            Md = robot->Md();
            M0 = robot->M0();
            pd.buildFrom(Md);
            // starting time of this motion [s]
            t0 = t;
        }

        // get current joint positions
        q = robot->jointPosition();
        cout << "Current joint position : " << q.t() << endl;

        // Direct Geometry for end-effector
        M = robot->fMe(q);  // matrix form
        p.buildFrom(M);     // translation + angle-axis form

        if(robot->mode() == ControlMode::POSITION_MANUAL)
        {
            // just check the Direct Geometric Model
            // DONE: fill the fMw function
            robot->checkPose(M);
        }


        else if(robot->mode() == ControlMode::VELOCITY_MANUAL)
        {
            // follow a given operational velocity
            v = robot->guiVelocityScrew(); // 6x1
            vpColVector fVe(6); // 6x1

            // Exercise 2
            // DONE: fill the fJw function
            // DONE: compute vCommand

            auto fRe = M.getRotationMatrix(); // get fRe rotation matrix

            // Build fVe
            ecn::putAt(fVe,(fRe*v.extract(0,3)),0); // Puts the 3x1 matrix (result from the product of fRe with the 3 first rows of v) in fVe
            ecn::putAt(fVe,(fRe*v.extract(3,3)),3); // Puts the 3x1 matrix (result from the product of fRe with the 3 last rows of v) in fVe

            vCommand = robot->fJe(q).pseudoInverse() * fVe;

            robot->setJointVelocity(vCommand);
        }

        else if(robot->mode() == ControlMode::DIRECT_P2P)
        {
            // find the Inverse Geometry to reach Md
            // DONE: fill the inverseGeometry function
            qf = robot->inverseGeometry(Md, q);
            robot->setJointPosition(qf);
        }


        else if(robot->mode() == ControlMode::POLYNOM_P2P)
        {
            // reach Md with polynomial joint trajectory
            // use q0 (initial position), qf (final), aMax and vMax

            // if reference has changed, compute new tf
            if(robot->newRef())
            {
                q0 = robot->inverseGeometry(M0, q);
                qf = robot->inverseGeometry(Md, q);

            }

            // Exercise 4 -- DONE
            auto dq = qf-q0;
            // check for all tfs, for all joints
            for(unsigned int i = 0; i < n; i++){
               auto tf1 = 15*abs(dq[i])/(8*vMax[i]);
               auto tf2 = sqrt(10*abs(dq[i])/(sqrt(3)*aMax[i]));

               tf = max({tf,tf1,tf2}); // at each cycle, compare tf, tf1 and tf2 to find the greatest
            }
            // subtract t0 because it changes at each point of the trajectory
            auto P = 10*(pow((t-t0)/tf,3))-15*(pow((t-t0)/tf,4))+6*(pow((t-t0)/tf,5));


            // TODO: compute qCommand from q0, qf, t, t0 and tf
            qCommand = q0 + P*dq;

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ControlMode::STRAIGHT_LINE_P2P)
        {
            // go from M0 to Md in 1 sec
            tf = 1;

            // TODO: compute qCommand from M0, Md, t, t0 and tf
            // use robot->intermediaryPose to build poses between M0 and Md

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ControlMode::VELOCITY_P2P)
        {
            // go to Md using operational velocity

            // TODO: compute joint velocity command



            robot->setJointVelocity(vCommand);
        }


    }
}
