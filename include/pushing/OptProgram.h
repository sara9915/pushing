#ifndef OPT_PROGH
#define OPT_PROGH

#include "geometry_msgs/WrenchStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/LinearMath/Transform.h"
#include "helper.h"
#include <ros/ros.h>
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"

//~ extern struct thread_data thread_data_array[1];
//~ extern std::vector<geometry_msgs::WrenchStamped> ft_wrenches;
extern pthread_mutex_t nonBlockMutex;

using namespace tf;
using namespace std;
using Eigen::MatrixXd;

void *rriMain(void *thread_arg);

#endif  
