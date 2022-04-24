// example_baxter_playfile_client
// wsn, September, 2016
// illustrates use of baxter_playfile_service

#include<ros/ros.h>
#include<baxter_playfile_nodes/playfileSrv.h>
#include <baxter_fk_ik/baxter_kinematics.h> 
#include <fstream>

int main(int argc, char** argv) {
    using namespace std;
    ros::init(argc, argv, "example_baxter_playfile_client"); // name this node 
    ros::NodeHandle nh;
    //create a client of playfile_service 


    Eigen::Vector3d p;
    Eigen::Affine3d A;
    Eigen::Affine3d Aapprox;
    Eigen::Affine3d toolAapprox;
    Eigen::Affine3d result;
    Eigen::Vector3d n_des,t_des,b_des;
    Eigen::Affine3d A_tool_wrt_flange_;
    Eigen::Affine3d A_tool_wrt_flange_inv_;
    Vectorq6x1 q_in;
    q_in<<0,0,0,0,0,0;

    Baxter_fwd_solver baxter_fwd_solver;
    Baxter_IK_solver baxter_ik_solver;
    Eigen::Matrix3d R_hand;
    Eigen::Vector3d O_hand;

    O_hand(0) = Lx_hand;
    O_hand(1) = 0.0;
    O_hand(2) = Lz_hand;
    
    R_hand(0, 0) = cos(theta_yaw_hand);
    R_hand(0, 1) = -sin(theta_yaw_hand); //% - sin(q(i))*cos(alpha);
    R_hand(0, 2) = 0.0; //
    R_hand(1, 0) = -R_hand(0, 1);
    R_hand(1, 1) = R_hand(0, 0); //
    R_hand(1, 2) = 0.0; //%
    R_hand(2,0) = 0.0;
    R_hand(2,1) = 0.0;
    R_hand(2,2) = 1.0;
       
    // set values for the tool transform, from flange to tool frame  
    //A_tool_to_flange_ = ...;
    A_tool_wrt_flange_.linear() = R_hand;
    A_tool_wrt_flange_.translation() = O_hand;
    A_tool_wrt_flange_inv_ = A_tool_wrt_flange_.inverse();

    /*b_des<<0,0,-1; //tool flange pointing down
    n_des<<1,0,0; //x-axis pointing forward...arbitrary
    t_des = b_des.cross(n_des); //consistent right-hand frame
    
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    Vectorq7x1 qsoln;    
    std::vector<Vectorq7x1> q_solns;
    Eigen::Vector3d p_des_tool;
    Eigen::Vector3d p_found;
    */
   Eigen::Quaterniond q;
   q.x() = 0.135;
   q.y() = 0.985;
   q.z() = -0.109;
   q.w() = 0.025;
   Eigen::Matrix3d R_des(q);
   Vectorq7x1 qsolnapprox;
   std::vector<Vectorq7x1> q_solns_approx;
   Eigen::Affine3d a_tool_des; // expressed in DH frame  
   Eigen::Affine3d a_flange_des;
   a_tool_des.linear() = R_des;
   Eigen::Vector3d p_des; //has 0.01 z offset because ik is off by that
   Eigen::Vector3d p_des_real;

   int nsolns;
    
   p_des[0] = 0.411;
   p_des[1] = 0.555;
   p_des[2] = 0.183 + 0.01; // test grasp pose; //z_high; //test approach pose
   a_tool_des.translation() = p_des;
   p_des_real[0] = 0.411;
   p_des_real[1] = 0.555;
   p_des_real[2] = 0.183;
   
   a_flange_des = a_tool_des*A_tool_wrt_flange_inv_; 
   
   /*
   a_tool_des_of_tool = a_tool_des*A_tool_wrt_flange_;
   a_tool_des = a_tool_des_of_tool*A_tool_wrt_flange_inv_;
   // Am able to convert from flange to tool to flange and get same!
   */

   
   nsolns = baxter_ik_solver.ik_solve_approx_wrt_torso(a_flange_des, q_solns_approx);
   cout<<nsolns<<endl;
   double minVal;
   int minValIndex;
   for(int solutionNumber = 0; solutionNumber<nsolns; solutionNumber++){
       qsolnapprox = q_solns_approx.at(solutionNumber);
       Aapprox = baxter_fwd_solver.fwd_kin_flange_wrt_torso_solve(qsolnapprox);
       toolAapprox = Aapprox*A_tool_wrt_flange_;
       if(solutionNumber==0){
           minVal = (toolAapprox.translation() - p_des).norm();
           minValIndex = 0;
       }
            
        else{
            if((toolAapprox.translation() - p_des_real).norm()<minVal)
            minVal = (toolAapprox.translation() - p_des_real).norm();
            minValIndex = solutionNumber;
        }
       //cout<<toolAapprox.translation()<<endl;
       //cout<<p_des<<endl;  
       
   }
   cout<<minVal<<endl;
   qsolnapprox = q_solns_approx.at(minValIndex);
   Aapprox = baxter_fwd_solver.fwd_kin_flange_wrt_torso_solve(qsolnapprox);
   toolAapprox = Aapprox*A_tool_wrt_flange_;






    ros::ServiceClient client = nh.serviceClient<baxter_playfile_nodes::playfileSrv>("playfile_service");
    baxter_playfile_nodes::playfileSrv playfile_srv_msg; //compatible service message
    //set the request to PRE_POSE, per the mnemonic defined in the service message
    double q0=qsolnapprox[0];
    double q1=qsolnapprox[1];
    double q2=qsolnapprox[2] ;
    double q3=qsolnapprox[3];
    double q4=qsolnapprox[4];
    double q5=qsolnapprox[5];
    double q6=qsolnapprox[6];
    double q7=3.0;


    client.waitForExistence();
    playfile_srv_msg.request.q0= q0;
    playfile_srv_msg.request.q1= q1;
    playfile_srv_msg.request.q2= q2;
    playfile_srv_msg.request.q3= q3;
    playfile_srv_msg.request.q4= q4;
    playfile_srv_msg.request.q5= q5;
    playfile_srv_msg.request.q6= q6;
    playfile_srv_msg.request.q7= q7;
    

    ROS_INFO("sending pre-pose command to playfile service: ");
    client.call(playfile_srv_msg);
    //blocks here until service call completes...
    ROS_INFO("service responded with code %d", playfile_srv_msg.response.return_code);
    return 0;
}

