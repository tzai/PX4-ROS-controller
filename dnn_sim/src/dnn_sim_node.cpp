#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <dnn_msgs/DnnOutput.h>

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#define LOCK_THRESHOLD 2


geometry_msgs::PoseStamped current_pose, platform_pose;
dnn_msgs::DnnOutput sim_output;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	current_pose = *msg;

	Eigen::Vector3d point1;
    tf::pointMsgToEigen(platform_pose.pose.position, point1);
    Eigen::Vector3d point2;
    tf::pointMsgToEigen(current_pose.pose.position, point2);

	Eigen::Vector3d delta = point1 - point2;
	sim_output.delta_x = delta[0];
	sim_output.delta_y = delta[1];
	sim_output.delta_z = delta[2];

	delta[2] = 0; // set Z value to zero so that norm only considers distance in XY plane
	if(delta.norm() < LOCK_THRESHOLD) {
		sim_output.has_lock = true;
	} else {
		sim_output.has_lock = false;
	}
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dnn_sim");
    ros::NodeHandle nh;

    // The location of the platform in the sim
    platform_pose.pose.position.x = 15;
    platform_pose.pose.position.y = 15;

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, poseCallback);
    ros::Publisher dnn_output_pub = nh.advertise<dnn_msgs::DnnOutput>("dnn/deltas", 10);
	ros::Rate rate(10.0);

	sim_output.delta_x = 0;
	sim_output.delta_y = 0;
	sim_output.delta_z = 0;
	sim_output.theta = 0;
	sim_output.has_lock = false;

	ROS_INFO("Starting DNN simulator");
    while(ros::ok()) {
		//local_pos_pub.publish(goto_pose);
    	dnn_output_pub.publish(sim_output);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}