
#include <occupancygrid_creator/occupancygrid_creator.h>

#include <memory>
#include <cmath>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>

OccupancygridCreator::OccupancygridCreator(ros::NodeHandle &node_handle)
    : nh_(node_handle)
{
    if (!loadConfig())
    {
        ros::shutdown();
    };
}

bool OccupancygridCreator::loadConfig()
{
    double frequency;
    double map_size_x;
    double map_size_y;
    double map_resolution;
    double map_center_x;
    double map_center_y;
    std::string map_id;
    std::string publish_topic;

    bool static_obstacle_use;
    std::vector<double> static_obstacle_x;
    std::vector<double> static_obstacle_y;
    std::vector<double> static_obstacle_radius;
    std::vector<std::string> receiving_obstacle_position_topics;

    std::string receiving_obstacle_position_gazebo_topic;

    if (!nh_.getParam("/gridmap/frequency", frequency))
    {
        return errorFunc("/gridmap/frequency");
    };
    if (!nh_.getParam("/gridmap/size_x", map_size_x))
    {
        return errorFunc("/gridmap/size_x");
    };
    if (!nh_.getParam("/gridmap/size_y", map_size_y))
    {
        return errorFunc("/gridmap/size_y");
    };
    if (!nh_.getParam("/gridmap/resolution", map_resolution))
    {
        return errorFunc("/gridmap/resolution");
    };
    if (!nh_.getParam("/gridmap/map_id", map_id))
    {
        return errorFunc("/gridmap/map_id");
    };
    if (!nh_.getParam("/gridmap/map_origin_x", map_center_x))
    {
        return errorFunc("/gridmap/map_origin_x");
    };
    if (!nh_.getParam("/gridmap/map_origin_y", map_center_y))
    {
        return errorFunc("/gridmap/map_origin_y");
    };
    if (!nh_.getParam("/gridmap/publish_topic", publish_topic))
    {
        return errorFunc("/gridmap/publish_topic");
    };

    if (!nh_.param("/obstacle_circle/create_static/use", static_obstacle_use, false))
    {
        defaultFunc("/obstacle_circle/create_static/use");
    };
    if (!nh_.param("/obstacle_circle/create_static/x", static_obstacle_x, {}))
    {
        defaultFunc("/obstacle_circle/create_static/x");
    };
    if (!nh_.param("/obstacle_circle/create_static/y", static_obstacle_y, {}))
    {
        defaultFunc("/obstacle_circle/create_static/y");
    };
    if (!nh_.param("/obstacle_circle/create_static/radius", static_obstacle_radius, {}))
    {
        defaultFunc("/obstacle_circle/create_static/radius");
    };

    if (!nh_.param("/obstacle_circle/receiving_position/use", receiving_obstacle_position_use_, false))
    {
        defaultFunc("/receiving_position/use");
    };
    if (!nh_.param("/obstacle_circle/receiving_position/topic", receiving_obstacle_position_topics, {}))
    {
        defaultFunc("/receiving_position/topic");
    };
    if (!nh_.param("/obstacle_circle/receiving_position/radius", receiving_obstacle_radius_, {}))
    {
        defaultFunc("/receiving_position/radius");
    };

    if (!nh_.param("/obstacle_circle/receiving_position_gazebo/use", receiving_obstacle_position_use_, false))
    {
        defaultFunc("/receiving_position_gazebo/use");
    };
    if (!nh_.param("/obstacle_circle/receiving_position_gazebo/topic", receiving_obstacle_position_gazebo_topic, std::string("temp")))
    {
        defaultFunc("/receiving_position_gazebo/topic");
    };
    if (!nh_.param("/obstacle_circle/receiving_position_gazebo/radius", receiving_obstacle_gazebo_radius_, 1.0))
    {
        defaultFunc("/receiving_position_gazebo/radius");
    };

    if (!nh_.param("/px4_control_interface/lab_bounds", lab_bounds_, {}))
    {
        defaultFunc("/px4_control_interface/lab_bounds");
    };

    gridmap_.header.stamp = ros::Time::now();
    gridmap_.header.frame_id = map_id;

    // all cells are initially unknown
    gridmap_.header.stamp = ros::Time::now();
    gridmap_.header.frame_id = map_id;
    // gridmap_.info.map_load_time = ros::Time(0);
    gridmap_.info.resolution = map_resolution;
    gridmap_.info.width = map_size_x / map_resolution;
    gridmap_.info.height = map_size_y / map_resolution;
    gridmap_.data.resize(gridmap_.info.width * gridmap_.info.height);

    gridmap_.info.origin.position.x = map_center_x - map_size_x / 2;
    gridmap_.info.origin.position.y = map_center_y - map_size_y / 2;

    /**
     * SETUP: Static obstacles
     */
    // Check if we need to create static obstacles
    if (static_obstacle_use)
    {
        if (!(static_obstacle_x.size() == static_obstacle_y.size()) || !(static_obstacle_x.size() == static_obstacle_radius.size()))
        {
            ROS_ERROR_STREAM("[Occupancygrid Creator]: Parameters static obstacles x, y and radius dont have the same length!");
            return false;
        }

        createStaticObstacles(static_obstacle_x, static_obstacle_y, static_obstacle_radius);
    }

    /**
     * SETUP: Receiving obstacles
     */
    if (!(receiving_obstacle_position_topics.size() == receiving_obstacle_radius_.size()))
    {
        ROS_ERROR_STREAM("[Occupancygrid Creator]: Parameters receiving_position/topic and receiving_position/radius dont have the same length!");
        return false;
    }

    state_msgs_stored_.resize(receiving_obstacle_position_topics.size());
    state_received_.resize(receiving_obstacle_position_topics.size());

    /**
     * SETUP: ROS
     */
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>(publish_topic, 1);

    subs_vector_.resize(receiving_obstacle_position_topics.size());
    if (receiving_obstacle_position_use_)
    {
        for (long unsigned int i = 0; i < receiving_obstacle_position_topics.size(); i++)
        {
            subs_vector_[i] = nh_.subscribe<nav_msgs::Odometry>(receiving_obstacle_position_topics[i], 1, std::bind(&OccupancygridCreator::callbackPositionObstacle, this, std::placeholders::_1, i));
        }
        // subs_vector_test_ = nh_.subscribe<nav_msgs::Odometry>(receiving_obstacle_position_topics[0], 1, &OccupancygridCreator::callbackPositionObstacle2, this);
    }

    if (receiving_obstacle_position_gazebo_use_)
    {
        sub_gazebo_ = nh_.subscribe(receiving_obstacle_position_gazebo_topic, 1, &OccupancygridCreator::callbackPositionObstacleGazebo, this);
    }

    // Add Lab bounds
    OccupancygridCreator::addLabBoundsToGrid();

    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &OccupancygridCreator::createMap, this);

    return true;
}

bool OccupancygridCreator::errorFunc(const std::string name)
{
    ROS_ERROR_STREAM("[Occupancygrid Creator]: Parameter \"" + name + "\" not defined!");
    return false;
}

void OccupancygridCreator::defaultFunc(const std::string name)
{
    ROS_ERROR_STREAM("[Occupancygrid Creator]: Parameter \"" + name + "\" not defined!");
}

void OccupancygridCreator::createMap(const ros::TimerEvent &event)
{
    if (receiving_obstacle_position_use_ && !(std::find(begin(state_received_), end(state_received_), true) == end(state_received_)))
    {
        ROS_WARN_STREAM("[Occupancygrid Creator]: Creating received Obstacles. ");
        // For each of the messages, place in the object
        nav_msgs::OccupancyGrid new_gridmap = gridmap_;
        std::vector<double> x;
        std::vector<double> y;
        for (long unsigned int i = 0; i < state_msgs_stored_.size(); i++)
        {
            if (!state_received_[i])
            {
                continue;
            };
            double x = state_msgs_stored_[i].pose.pose.position.x;
            double y = state_msgs_stored_[i].pose.pose.position.y;
            placeObstacleInGrid(new_gridmap, x, y, receiving_obstacle_radius_[i]);
        }
        pub_map_.publish(new_gridmap);

        return;
    }
    else if (receiving_obstacle_position_gazebo_use_ && state_gazebo_received_)
    {
        ROS_WARN_STREAM("[Occupancygrid Creator]: Creating received Gazebo Obstacles. ");
        // For each of the messages, place in the object
        nav_msgs::OccupancyGrid new_gridmap = gridmap_;
        std::vector<double> x;
        std::vector<double> y;
        for (long unsigned int i = 0; i < state_msgs_gazebo_stored_.name.size(); i++)
        {
            if (state_msgs_gazebo_stored_.name[i].find("cylinder") == std::string::npos)
            {
                continue;
            };

            ROS_WARN_STREAM("found :" << state_msgs_gazebo_stored_.name[i]);
            double x = state_msgs_gazebo_stored_.pose[i].position.x;
            double y = state_msgs_gazebo_stored_.pose[i].position.y;
            placeObstacleInGrid(new_gridmap, x, y, receiving_obstacle_gazebo_radius_);
        }

        pub_map_.publish(new_gridmap);
        return;
    }

    pub_map_.publish(gridmap_);
}

void OccupancygridCreator::createStaticObstacles(std::vector<double> x, std::vector<double> y, std::vector<double> radius)
{
    // first change to integer parameters of grid structure
    for (long unsigned int i = 0; i < x.size(); i++)
    {
        placeObstacleInGrid(gridmap_, x[i], y[i], radius[i]);
    }
}

void OccupancygridCreator::placeObstacleInGrid(nav_msgs::OccupancyGrid &gridmap, double x_cur, double y_cur, double radius_cur)
{
    double rad_steps = std::atan2(gridmap_.info.resolution, radius_cur);

    for (double radians = 0; radians < 2 * 3.14159265358979323846; radians = radians + rad_steps)
    {
        double x_on_circle = std::cos(radians) * radius_cur;
        double y_on_circle = std::sin(radians) * radius_cur;

        // convert to map indexes
        int x_on_grid = (x_on_circle + x_cur - gridmap.info.origin.position.x) / gridmap.info.resolution;
        int y_on_grid = (y_on_circle + y_cur - gridmap.info.origin.position.y) / gridmap.info.resolution;

        gridmap.data[x_on_grid + y_on_grid * gridmap_.info.height] = 100;
    }
}

void OccupancygridCreator::addLabBoundsToGrid()
{
    // Add the lab bounds as obstacles to the grid map
    // x_min is the first index of free space, x_max is the last index of free space
    int x_min, x_max, y_min, y_max;
    x_min = (-lab_bounds_[0] - gridmap_.info.origin.position.x) / gridmap_.info.resolution;
    x_max = (lab_bounds_[0] - gridmap_.info.origin.position.x) / gridmap_.info.resolution;
    y_min = (-lab_bounds_[1] - gridmap_.info.origin.position.y) / gridmap_.info.resolution;
    y_max = (lab_bounds_[1] - gridmap_.info.origin.position.y) / gridmap_.info.resolution;

    // int width = (int)gridmap_.info.width;
    // int height = (int)gridmap_.info.height;

    // Print the bounds
    ROS_INFO_STREAM("[Occupancygrid Creator]: Lab bounds: " << x_min << " " << x_max << " " << y_min << " " << y_max);
    ROS_INFO_STREAM("[Occupancygrid Creator]: Gridmap bounds: " << gridmap_.info.width << " " << gridmap_.info.height);

    // Add occupied cells to the grid map
    for (int i = 0; i < (int)gridmap_.info.width; i++)
    {
        for (int j = 0; j < (int)gridmap_.info.height; j++)
        {
            if (i < x_min || i > x_max || j < y_min || j > y_max)
            {
                gridmap_.data[i + j * gridmap_.info.height] = 100;
            }
        }
    }
}

void OccupancygridCreator::callbackPositionObstacle(const nav_msgs::Odometry::ConstPtr &msg, const long unsigned int i)
{
    state_msgs_stored_[i] = *msg;
    state_received_[i] = true;
}

void OccupancygridCreator::callbackPositionObstacleGazebo(const gazebo_msgs::ModelStates &msg)
{
    state_msgs_gazebo_stored_ = msg;
    state_gazebo_received_ = true;
}
