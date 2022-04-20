// reachability_from_above.cpp
// wsn, September 2016
// compute reachability, w/ z_tool_des pointing down;
// distance from gripper frame to flange frame is 0.1577
// cafe table is 0.79m above ground plane

// w/ Baxter on pedestal, torso is 0.93m above ground plane
// therefore, expect when fingers touch table, flange is at 0.79+0.1577 
// i.e., 0.0177m above torso
// search for reachability for flange over x_range = [0.4,1] , y_range= [-1,1] at z_range =[0,0.1]


#include <baxter_fk_ik/baxter_kinematics.h> 
#include <fstream>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_reachability");
    Eigen::Vector3d p;
    Eigen::Affine3d A;
    Eigen::Affine3d Aapprox;
    Eigen::Affine3d result;
    Eigen::Vector3d n_des,t_des,b_des;
    Eigen::Quaterniond q;
    Vectorq6x1 q_in;
    q_in<<0,0,0,0,0,0;

    Baxter_fwd_solver baxter_fwd_solver;
    Baxter_IK_solver baxter_ik_solver;

    b_des<<0,0,-1; //tool flange pointing down
    n_des<<1,0,0; //x-axis pointing forward...arbitrary
    t_des = b_des.cross(n_des); //consistent right-hand frame
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    std::vector<Vectorq7x1> q_solns;
    Vectorq7x1 qsolnapprox;
    Vectorq7x1 qsoln;
    std::vector<Vectorq7x1> q_solns_approx;
    Eigen::Affine3d a_tool_des; // expressed in DH frame  
    a_tool_des.linear() = R_des;
    //a_tool_des.translation() << x_des,0,0;
    double w_err_norm; 
    Eigen::Vector3d p_des;
    Eigen::Vector3d p_found;
    int nsolns;
    
   p_des[0] = 0.45;
   p_des[1] = 0.55;
   p_des[2] = 0.0; // test grasp pose; //z_high; //test approach pose
   a_tool_des.translation() = p_des;

   nsolns = baxter_ik_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns_approx);
   cout<<nsolns<<endl;
   for(int solutionNumber = 0; solutionNumber<nsolns; solutionNumber++){
       qsolnapprox = q_solns_approx.at(solutionNumber);
       Aapprox = baxter_fwd_solver.fwd_kin_flange_wrt_torso_solve(qsolnapprox);
       cout << (Aapprox.translation() - p_des).norm()<<endl;     
   }
   
    return 0;
}
