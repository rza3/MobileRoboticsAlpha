#include <ros/ros.h>
#include "double_vec_srv/DoubleSrv.h"


using namespace std;

int main(int argc , char **argv)
{
    ros::init(argc, argv, "heading_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<double_vec_srv::DoubleSrv>("stdr_rotation_service");
    double_vec_srv::DoubleSrv srv;
    double desired_heading;
    desired_heading=1;// should be the value of accumulation of rotation angle of robot.


    // while(ros::ok()){
       // cout<<"enter  a desired heading:";
       //  cin>>desired_heading;

        srv.request.vec_of_doubles.resize(1);
        srv.request.vec_of_doubles[0]=desired_heading;
        client.call(srv);
    //}

}