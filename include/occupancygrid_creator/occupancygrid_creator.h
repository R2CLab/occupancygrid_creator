#ifndef OCCUPANCYGRID_CREATOR_OCCUPANCYGRID_CREATOR_H
#define OCCUPANCYGRID_CREATOR_OCCUPANCYGRID_CREATOR_H

#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>


class OccupancygridCreator
{
public:
    ros::NodeHandle nh_;
    ros::Timer timer_;

    // ros messages
    nav_msgs::OccupancyGrid gridmap_;
    std::vector<geometry_msgs::PoseStamped> state_msgs_stored_;
    geometry_msgs::PoseStamped state_msgs_stored_2_;

    // ros publisher
    ros::Publisher pub_map_;

    // ros subscribers
    std::vector<ros::Subscriber> subs_vector_;
    ros::Subscriber subs_vector_test_;
    ros::Subscriber sub_gazebo_;


    // stored parameters
    //// received obstacle
    bool receiving_obstacle_position_use_;
    std::vector<double> receiving_obstacle_radius_;
    std::vector<bool>state_received_;
    //// received gazebo obstacle
    bool receiving_obstacle_position_gazebo_use_;
    double receiving_obstacle_gazebo_radius_;
    bool state_gazebo_received_;
    gazebo_msgs::ModelStates state_msgs_gazebo_stored_;

    OccupancygridCreator(ros::NodeHandle &node_handle);

    bool loadConfig();

    bool errorFunc(const std::string name);

    void defaultFunc(const std::string name);

    void createMap(const ros::TimerEvent& event);

    void createStaticObstacles(std::vector<double> x, std::vector<double> y, std::vector<double> radius);

    void placeObstacleInGrid(nav_msgs::OccupancyGrid &gridmap, double x_cur, double y_cur, double radius_cur);

    void callbackPositionObstacle(const geometry_msgs::PoseStamped::ConstPtr &msg, const long unsigned int i);
    
    void callbackPositionObstacle2(const geometry_msgs::PoseStamped msg);

    void callbackPositionObstacleGazebo(const gazebo_msgs::ModelStates &msg);
};

#endif
