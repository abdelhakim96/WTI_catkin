/**
*    \author Mohit Mehndiratta
*    \date   2021
*/

#include <acado_code_generation.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>

int main()
{
    USING_NAMESPACE_ACADO

    // Variables:
    DifferentialState x;  // the body position w.r.t X_I
    DifferentialState y;  // the body position w.r.t Y_I
    DifferentialState z;  // the body position w.r.t Z_I
    DifferentialState u;  // the translation velocity along X_B
    DifferentialState v;  // the translation velocity along Y_B
    DifferentialState w;  // the translation velocity along Z_B
    DifferentialState q_x;
    DifferentialState q_y;
    DifferentialState q_z;
    DifferentialState q_w;

    OnlineData Fx_dist;  // the external disturbance force along X_B
    OnlineData Fy_dist;  // the external disturbance force along Y_B
    OnlineData Fz_dist;  // the external disturbance force along Z_B

    OnlineData            p_F_x, p_F_y, p_F_z;
    

    DifferentialState aux_state_px, aux_state_py, aux_state_pz;
    DifferentialState aux_state_nx, aux_state_ny, aux_state_nz;
    Control wx;  // the roll rate
    Control wy;  // the pitch rate
    Control wz;  // the yaw rate
    Control T;      // the external force along Z_B
    


    const double q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;
    
    q_B_C_x = 0;
    q_B_C_y = 0;
    q_B_C_z = 0;
    q_B_C_w = 1;

    const double m = 3.8;   // kg
    const double g = 9.81;  // m/s^2
    const double d = 10.0;   //m

    const double epsilon = 0.1; 
    // Model equations:
    DifferentialEquation f;


  // System Dynamics
  f << dot(x) ==  v_x;
  f << dot(y) ==  v_y;
  f << dot(w) ==  v_z;
  f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
   f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(v_x) ==  2 * ( q_w * q_y + q_x * q_z ) * T*(1 / m);
  f << dot(v_y) ==  2 * ( q_y * q_z - q_w * q_x ) * T*(1 / m);
  f << dot(v_z) ==  ( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * T*(1 / m) - g;


  // Intermediate states to calculate point of interest projection!
  IntermediateState intSx = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F_z-p_z));
  IntermediateState intSy = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w))*(p_F_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F_x-p_x));
  IntermediateState intSz = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w))*(p_F_y-p_y));



    // Reference functions and weighting matrices:
    Function h, hN;
    h << x << y << z << q_x << q_y << q_z << q_w <<  v_x << v_y << v_z << intSx/(intSz + epsilon) << intSy/(intSz + epsilon)  << p_rate << q_rate << r_rate << Fz;
    hN << x << y << z << q_x << q_y << q_z << q_w << v_x << v_y << v_z;

    BMatrix W = eye<bool>(h.getDim());
    BMatrix WN = eye<bool>(hN.getDim());

    //
    // Optimal Control Problem
    //
    double N = 30;
    double Ts = 0.01;
    OCP ocp(0.0, N * Ts, N);

    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    ocp.subjectTo(-100 * M_PI / 180 <= p_rate <= 100 * M_PI / 180);
    ocp.subjectTo(-100 * M_PI / 180 <= q_rate <= 100 * M_PI / 180);
    ocp.subjectTo(-100 * M_PI / 180 <= r_rate <= 100 * M_PI / 180);
    ocp.subjectTo(0.3 * m * g <= Fz <= 2 * m * g);

    // Export the code:
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_RK4);
    mpc.set(NUM_INTEGRATOR_STEPS, 2 * N);

    mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(MAX_NUM_QP_ITERATIONS, 1000);

    // 	mpc.set( SPARSE_QP_SOLUTION, SPARSE_SOLVER );
    // 	mpc.set( QP_SOLVER, QP_QPDUNES );

    mpc.set(HOTSTART_QP, YES);

    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, YES);  // Possible to Change Constraints Afterwards (only with qpOASES)

    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    // Optionally set custom module name:
    mpc.set(CG_MODULE_NAME, "nmpc");
    mpc.set(CG_MODULE_PREFIX, "NMPC");

    std::string path = ros::package::getPath("acado_ccode_generation");
    std::string path_dir = path + "/solver/quaternion_point_traj_ccode";
    ROS_INFO("%s", path_dir.c_str());

    try
    {
        ROS_WARN("TRYING TO EXPORT");
        if (mpc.exportCode(path_dir) != SUCCESSFUL_RETURN)
            ROS_ERROR("FAIL EXPORT CODE");
    }
    catch (...)
    {
        ROS_ERROR("FAIL TO EXPORT");
    }

    mpc.printDimensionsQP();

    ROS_WARN("DONE CCODE");

    return EXIT_SUCCESS;
}
