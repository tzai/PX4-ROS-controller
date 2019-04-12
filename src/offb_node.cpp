/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include "radius_controller.h"
#include <fstream> // file IO for debug

enum OffboardState {
    USER_CTRL, // user has full control and can put drone in other flight modes (altitude, position, manual, etc)
    DNN_NO_LOCK, // offboard node is in control, but DNN does not have lock. controller is navitating to GPS coordinates of landing platform
    DNN_LOCK, // DNN has lock, getting ready to land
    DNN_LOCK_LANDING // DNN has lock, actively landing
};

#define NUM_WAYPOINTS 3
#define UPDATE_RATE 20.0

mavros_msgs::State current_state;
OffboardState offboard_state = USER_CTRL;
dnn_msgs::DnnOutput dnn_output;
geometry_msgs::PoseStamped current_pose, current_est_pose, goto_pose, platform_pose, test_waypoints[NUM_WAYPOINTS], last_save_pose;
ros::Time last_log_time, dnn_lock_time, last_save_time;
double dnn_trajectory_err = 0;
int waypoint_index = 0;

// trajectory stuff
geometry_msgs::Vector3 lin_velocity;
std::deque<std::shared_ptr<std::array<double, 3>>> trajectory;

// thresholds
double z_axis_threshold = cos(M_PI/6); // when z axis is off by 30 degrees from vertical
double position_threshold = 0.5;
double dnn_lock_wait_time = 4.0;
double dnn_error_threshold = 0.3;
double pose_save_period = 2.0;
double z_landing_vel = 0.3;

inline double getVectorDistance(Eigen::Vector3d& point1, Eigen::Vector3d& point2) {
    return (point1-point2).norm();
}

inline double getPoseDistance(geometry_msgs::PoseStamped& pose1, geometry_msgs::PoseStamped& pose2) {
    Eigen::Vector3d point1;
    tf::pointMsgToEigen(pose1.pose.position, point1);
    Eigen::Vector3d point2;
    tf::pointMsgToEigen(pose2.pose.position, point2);
    
    return getVectorDistance(point1, point2);
}

void computeTrajectory(std::deque<std::shared_ptr<std::array<double, 3>>> &trajectory, geometry_msgs::PoseStamped &start_pose, geometry_msgs::PoseStamped &end_pose, double update_rate) {
    Eigen::Vector3d start_pos, end_pos, delta_pos;
    tf::pointMsgToEigen(start_pose.pose.position, start_pos);
    tf::pointMsgToEigen(end_pose.pose.position, end_pos);

    delta_pos = (end_pos - start_pos);
    ROS_INFO("Start pos = x:%.3lf y:%.3lf z:%.3lf, end pos = x:%.3lf y:%.3lf z:%.3lf, delta = x:%.3lf y:%.3lf z:%.3lf",
        start_pos[0], start_pos[1], start_pos[2],
        end_pos[0], end_pos[1], end_pos[2],
        delta_pos[0], delta_pos[1], delta_pos[2]);
    ROS_INFO("Using linear velocities = x:%.3lf y:%.3lf", lin_velocity.x, lin_velocity.y);

    // ported MATLAB code to generate cubic trajectory
    // get rough distance estimation and calculate tau for segment based on max velocity
    double tau_seg = delta_pos[2];
    double z_step = -z_landing_vel / update_rate;
    double tau_step = z_step / delta_pos[2];

    // Matrix used for solving for polynomial coefficients
    Eigen::Matrix2d A;
    Eigen::Vector2d b, x;
    A << 1,1, 3,2;
    // calculate parametric x
    double coeff_x[3];
    coeff_x[0] = lin_velocity.x*tau_seg/delta_pos[0];
    b << 1 - coeff_x[0], -coeff_x[0];
    x = A.colPivHouseholderQr().solve(b);
    coeff_x[2] = x[0];
    coeff_x[1] = x[1];

    // parametrix y
    double coeff_y[3];
    coeff_y[0] = lin_velocity.y*tau_seg/delta_pos[1];
    b << 1 - coeff_y[0], -coeff_y[0];
    x = A.colPivHouseholderQr().solve(b);
    coeff_y[2] = x[0];
    coeff_y[1] = x[1];

    // max # of points is total z distance divided by step in z axis
    int max_points = delta_pos[2] / z_step;
    //ROS_INFO("delta z: %.2lf, z_step: %.2lf, max_points: %d", delta_pos[2], z_step, max_points);
    trajectory.clear();
    for(int i = 1; i < max_points; ++i) {
        // if, due to integer steps, next waypoint will overshoot platform, stop here
        if(abs(i*z_step) > abs(delta_pos[2])) {
            ROS_INFO("passed z delta. breaking out of for loop");
            break;
        }

        trajectory.push_back(std::shared_ptr<std::array<double, 3>>(new std::array<double, 3>()));
        (*trajectory.back())[0] = start_pos[0] + delta_pos[0]*(coeff_x[2]*pow(i*tau_step, 3) + coeff_x[1]*pow(i*tau_step, 2) + coeff_x[0]*i*tau_step);
        (*trajectory.back())[1] = start_pos[1] + delta_pos[1]*(coeff_y[2]*pow(i*tau_step, 3) + coeff_y[1]*pow(i*tau_step, 2) + coeff_y[0]*i*tau_step);
        (*trajectory.back())[2] = start_pos[2] + i*z_step;
        //ROS_INFO("i=%d:%4.2lf",i,(*trajectory.back())[0]);
        //ROS_INFO("point at i=%d: x:%.3lf y:%.3lf z:%.3lf", i, next_pose.position.x, next_pose.position.y, next_pose.position.z);
    }
}

void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    lin_velocity = msg->twist.linear;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // Update state, TODO: add spinlock if threaded?
    current_pose = *msg;
    // end of lock

    //check if drone is out of the horizontal plane by more than the allowed threshold
    Eigen::Quaterniond current_orientation;
    tf::quaternionMsgToEigen(current_pose.pose.orientation, current_orientation);
    Eigen::Matrix3d rotation_mat = current_orientation.toRotationMatrix();
    // compares vertical component of the Z axis of the rotation matrix to the defined threshold
    if((offboard_state == DNN_LOCK || offboard_state == DNN_LOCK_LANDING) && rotation_mat(2, 2) < z_axis_threshold) {
        offboard_state = DNN_NO_LOCK;
        waypoint_index = 0; // reset to first waypoint
        ROS_INFO("Z axis threshold exceeded! Reverting to DNN_NO_LOCK state");
    }

    if(ros::Time::now() - last_log_time > ros::Duration(1.0)) {
        Eigen::Vector3d euler_angles = rotation_mat.eulerAngles(0,1,2);

        ROS_INFO("Pos:%4.2f, %4.2f, %4.2f, Att:%4.2f, %4.2f, %4.2f",
                 current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                 angles::to_degrees(euler_angles[0]), angles::to_degrees(euler_angles[1]), angles::to_degrees(euler_angles[2]));

        last_log_time = ros::Time::now();
    }

    // save a successful lock pose every <pose_save_period> seconds, so that we can try to recover to a known pose if lock is lost
    if((DNN_LOCK || DNN_LOCK_LANDING) && ros::Time::now() - last_save_time > ros::Duration(pose_save_period)) {
        last_save_pose = current_pose;
        last_save_time = ros::Time::now();
    }
}

void dnnCallback(const dnn_msgs::DnnOutput::ConstPtr &msg) {
    dnn_output = *msg;

    // for now, always trust DNN. TODO: make this check validate that we are close to the platform according to the GPS as well
    if(offboard_state == DNN_NO_LOCK && dnn_output.has_lock) {
        offboard_state = DNN_LOCK;
    } else if((offboard_state == DNN_LOCK || offboard_state == DNN_LOCK_LANDING) && !dnn_output.has_lock) { // if lock has been lost
        offboard_state = DNN_NO_LOCK;
    }

    // if we are landing and error is outside of the allowed tolerance, compute trajectory with updated pose info
    if(offboard_state == DNN_LOCK_LANDING) {
        Eigen::Vector3d goto_point, vision_point;
        tf::pointMsgToEigen(goto_pose.pose.position, goto_point);
        tf::pointMsgToEigen(platform_pose.pose.position, vision_point);
        vision_point[0] -= dnn_output.delta_x;
        vision_point[1] -= dnn_output.delta_y;
        vision_point[2] -= dnn_output.delta_z;
        // calculate error in XY plane only
        Eigen::Vector3d goto_xy, vision_xy;
        goto_xy << goto_point[0], goto_point[1], 0;
        vision_xy << vision_point[0], vision_point[1], 0;
        dnn_trajectory_err = getVectorDistance(goto_xy, vision_xy);
        current_est_pose.pose.position.x = vision_point[0];
        current_est_pose.pose.position.y = vision_point[1];
        current_est_pose.pose.position.z = vision_point[2];
        current_est_pose.pose.orientation = current_pose.pose.orientation;
        
        if(dnn_trajectory_err > dnn_error_threshold) {
            computeTrajectory(trajectory, current_est_pose, platform_pose, UPDATE_RATE);
            ROS_ASSERT(trajectory.front() != NULL); // make sure that first element is valid
            ROS_INFO("DNN error: %4.2lf. Computed trajectory with %ld intermediate waypoints", dnn_trajectory_err, trajectory.size());
        }
    }
}

void joystickCallback(const sensor_msgs::Joy::ConstPtr &msg) {
    float mov_stick_updown = msg->axes[0];
    float mov_stick_leftright = msg->axes[1];
    float pos_stick_updown = msg->axes[2];
    float pos_stick_leftright = msg->axes[3];

    ROS_INFO("JOY_VALS: l=%2.2f, a=%2.2f, y=%2.2f, alt=%2.2f",
              mov_stick_updown, mov_stick_leftright, pos_stick_leftright, pos_stick_updown);
}

inline float doExpSmoothing(float cur, float prev, float factor) {
    return factor * cur + (1.0f - factor) * prev;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    std::ofstream trajectory_log;
    char log_name[32];

    while(ros::Time::now().toSec() == 0) { // in a simulation, Time::now() returns zero until the first /clock msg is sent, so wait for it
    }
    sprintf(log_name, "trajectory_log_%lf.log", ros::Time::now().toSec());
    trajectory_log.open(log_name);
    trajectory_log << "time,goto_x,goto_y,goto_z,error\n";

    double z_angle_threshold;
    std::vector<double> platform_pos;
    // load configurable params
    nh.param("/offb_node/z_angle_threshold", z_angle_threshold, M_PI/6.0);
    z_axis_threshold = cos(z_angle_threshold);
    nh.param("/offb_node/position_threshold", position_threshold, 0.5);
    nh.param("/offb_node/dnn_lock_wait_time", dnn_lock_wait_time, 2.0);
    nh.param("/offb_node/dnn_error_threshold", dnn_error_threshold, 0.3);
    nh.param("/offb_node/z_landing_vel", z_landing_vel, 0.7);
    if(!nh.getParam("/offb_node/platform_pos", platform_pos)) {
        ROS_ERROR("Error: required parameter 'platform_pos' not available!");
        return -1;
    }
    ROS_INFO_NAMED("Param info","Using given platform location: x=%lf, y=%lf, z=%lf", platform_pos[0], platform_pos[1], platform_pos[2]);
    platform_pose.pose.position.x = platform_pos[0];
    platform_pose.pose.position.y = platform_pos[1];
    platform_pose.pose.position.z = platform_pos[2];

    ROS_INFO("Started offb_node");

    const int QUEUE_SIZE = 10;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", QUEUE_SIZE, stateCallback);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", QUEUE_SIZE, poseCallback);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", QUEUE_SIZE, twistCallback);
    ros::Subscriber dnn_output_sub = nh.subscribe<dnn_msgs::DnnOutput>("dnn/deltas", QUEUE_SIZE, dnnCallback);

    // Subscribe to a JOY (joystick) node if available
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", QUEUE_SIZE, joystickCallback);
    if(joy_sub){
        ROS_INFO("Subscribed to /joy topic (joystick)");
    }

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", QUEUE_SIZE);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(UPDATE_RATE);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    Eigen::Quaterniond rotation_q;
    rotation_q = Eigen::AngleAxis<double>(M_PI/2.0, Eigen::Vector3d::UnitZ());

    test_waypoints[0].pose.position.x = 0;
    test_waypoints[0].pose.position.y = 0;
    test_waypoints[0].pose.position.z = 5;

    test_waypoints[1].pose.position.x = 0;
    test_waypoints[1].pose.position.y = 15;
    test_waypoints[1].pose.position.z = 10;
    //tf::quaternionEigenToMsg(rotation_q, test_waypoints[1].pose.orientation);

    test_waypoints[2].pose.position.x = 15;
    test_waypoints[2].pose.position.y = 15;
    test_waypoints[2].pose.position.z = 10;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(goto_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    last_log_time = ros::Time::now();

    offboard_state = USER_CTRL;
    ROS_INFO("Starting main spin loop");
    while(ros::ok()){
        switch(offboard_state) {
        case USER_CTRL:
            if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                        offboard_state = DNN_NO_LOCK;
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            break;
        case DNN_NO_LOCK:
            goto_pose = test_waypoints[waypoint_index];
            if(getPoseDistance(current_pose, goto_pose) <= position_threshold) {
                if(waypoint_index < NUM_WAYPOINTS - 1) {
                    waypoint_index++;
                    ROS_INFO("Reached waypoint. Targeting next waypoint");
                }
            }
            break;
        case DNN_LOCK:
            goto_pose = test_waypoints[NUM_WAYPOINTS - 1];
            if(ros::Time::now() - dnn_lock_time > ros::Duration(dnn_lock_wait_time)) {
                offboard_state = DNN_LOCK_LANDING;
                ROS_INFO("Achieved DNN_LOCK for %f seconds. Entering DNN_LOCK_LANDING state", dnn_lock_wait_time);
            }
            break;
        case DNN_LOCK_LANDING:
            // get next intermediate waypoint for the trajetory
            //ROS_DEBUG("trajectory # items: %d, is empty: %d", (int) trajectory.size(), trajectory.empty());
            if(!trajectory.empty()) {
                std::shared_ptr<std::array<double, 3>> pos = trajectory.front();
                ROS_ASSERT(pos != NULL);
                goto_pose.pose.position.x = (*pos)[0];
                goto_pose.pose.position.y = (*pos)[1];
                goto_pose.pose.position.z = (*pos)[2];
                trajectory.pop_front();
                trajectory_log << boost::format("%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf%.3lf\n")
                    % ros::Time::now().toSec() 
                    % goto_pose.pose.position.x % goto_pose.pose.position.y % goto_pose.pose.position.z
                    % dnn_trajectory_err
                    % current_pose.pose.position.x % current_pose.pose.position.y % current_pose.pose.position.z;
            } else {
                ROS_INFO("ran out of trajectory points while landing. This shouldn't happen while DNN is running");
            }
            break;
        
        default:
            ROS_ERROR("Unknown controller state in main loop");
            trajectory_log.close();
            return -1;
        }

        local_pos_pub.publish(goto_pose);

        ros::spinOnce();
        rate.sleep();
    }

    trajectory_log.close();
    return 0;
}