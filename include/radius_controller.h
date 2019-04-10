#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <dnn_msgs/DnnOutput.h>
#include <sensor_msgs/Joy.h>

#include <math.h>

#include <tf/tf.h>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>

#include <deque>
#include <array>