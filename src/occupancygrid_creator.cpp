
#include <occupancygrid_creator/occupancygrid_creator.h>


#include <memory>
#include <cmath>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

OccupancygridCreator::OccupancygridCreator(ros::NodeHandle &node_handle)
    :nh_(node_handle)
{
    if (!loadConfig()){ros::shutdown();};


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
    std::vector<double> static_circles_x;
    std::vector<double> static_circles_y;
    std::vector<double> static_circles_radius;
    std::vector<double> static_squares_x;
    std::vector<double> static_squares_y;
    std::vector<double> static_squares_length;
    std::vector<double> static_squares_width;
    std::vector<double> static_squares_orientation;
    std::vector<double> static_squares_line_thickness;



    std::vector<std::string> receiving_obstacle_position_topics;

    std::string receiving_obstacle_position_gazebo_topic;


    if (!nh_.getParam("/gridmap/frequency", frequency)){return errorFunc("/gridmap/frequency");};
    if (!nh_.getParam("/gridmap/size_x", map_size_x)){return errorFunc("/gridmap/size_x");};
    if (!nh_.getParam("/gridmap/size_y", map_size_y)){return errorFunc("/gridmap/size_y");};
    if (!nh_.getParam("/gridmap/resolution", map_resolution)){return errorFunc("/gridmap/resolution");};
    if (!nh_.getParam("/gridmap/map_id", map_id)){return errorFunc("/gridmap/map_id");};
    if (!nh_.getParam("/gridmap/map_origin_x", map_center_x)){return errorFunc("/gridmap/map_origin_x");};
    if (!nh_.getParam("/gridmap/map_origin_y", map_center_y)){return errorFunc("/gridmap/map_origin_y");};
    if (!nh_.getParam("/gridmap/publish_topic", publish_topic)){return errorFunc("/gridmap/publish_topic");};

    if (!nh_.param("/obstacles/create_static/use", static_obstacle_use, false)){defaultFunc("/obstacle/use");};

    if (!nh_.param("/obstacles/create_static/circles/x_center", static_circles_x, {})){defaultFunc("/obstacle/create_static/circle/x_center");};
    if (!nh_.param("/obstacles/create_static/circles/y_center", static_circles_y, {})){defaultFunc("/obstacle/create_static/circle/y_center");};
    if (!nh_.param("/obstacles/create_static/circles/radius", static_circles_radius, {})){defaultFunc("/obstacle/create_static/circle/radius");};

    if (!nh_.param("/obstacles/create_static/squares/x_center", static_squares_x, {})){defaultFunc("/obstacle_create_static/square/x_center");};
    if (!nh_.param("/obstacles/create_static/squares/y_center", static_squares_y, {})){defaultFunc("/obstacle_create_static/square/y_center");};
    if (!nh_.param("/obstacles/create_static/squares/length", static_squares_length, {})){defaultFunc("/obstacle_create_static/square/length");};
    if (!nh_.param("/obstacles/create_static/squares/width", static_squares_width, {})){defaultFunc("/obstacle_create_static/square/width");};
    if (!nh_.param("/obstacles/create_static/squares/orientation_deg", static_squares_orientation, {})){defaultFunc("/obstacle_create_static/square/rotation");};
    if (!nh_.param("/obstacles/create_static/squares/line_thickness", static_squares_line_thickness, {})){defaultFunc("/obstacle_create_static/square/line_thickness");};

    if (!nh_.param("/obstacles/receiving_position/use", receiving_obstacle_position_use_, false)){defaultFunc("/receiving_position/use");};
    if (!nh_.param("/obstacles/receiving_position/topic", receiving_obstacle_position_topics, {})){defaultFunc("/receiving_position/topic");};
    if (!nh_.param("/obstacles/receiving_position/radius", receiving_obstacle_radius_, {})){defaultFunc("/receiving_position/radius");};

    if (!nh_.param("/obstacles/receiving_position_gazebo/use", receiving_obstacle_position_gazebo_use_, false)){defaultFunc("/receiving_position_gazebo/use");};
    if (!nh_.param("/obstacles/receiving_position_gazebo/topic", receiving_obstacle_position_gazebo_topic, std::string("temp"))){defaultFunc("/receiving_position_gazebo/topic");};
    if (!nh_.param("/obstacles/receiving_position_gazebo/radius", receiving_obstacle_gazebo_radius_, 1.0)){defaultFunc("/receiving_position_gazebo/radius");};



    gridmap_.header.stamp = ros::Time::now();
    gridmap_.header.frame_id = map_id;
    
    //all cells are initially unknown
    gridmap_.header.stamp = ros::Time::now();
    gridmap_.header.frame_id = map_id;
    //gridmap_.info.map_load_time = ros::Time(0);
    gridmap_.info.resolution = map_resolution;
    gridmap_.info.width = map_size_x/map_resolution;
    gridmap_.info.height = map_size_y/map_resolution;
    gridmap_.data.resize(gridmap_.info.width * gridmap_.info.height);

    gridmap_.info.origin.position.x = map_center_x - map_size_x / 2;
    gridmap_.info.origin.position.y = map_center_y - map_size_y / 2;

    //here we create the image
    cv::Mat occupancy_image(gridmap_.info.width, gridmap_.info.height, CV_8UC1, gridmap_.data.data());

    /**
     * SETUP: Static obstacles
    */
    // Check if we need to create static obstacles
    if (static_obstacle_use)
    {
        if (!(static_circles_x.size() == static_circles_y.size()) || !(static_circles_x.size() == static_circles_radius.size()))
        {
            ROS_ERROR_STREAM("[Occupancygrid Creator]: Parameters static circles x, y and radius dont have the same length!");
            return false;
        }

        createStaticObstacles(static_circles_x, static_circles_y, static_circles_radius);


        if (!(static_squares_x.size() == static_squares_y.size()) || !(static_squares_x.size() == static_squares_length.size()) ||
            !(static_squares_x.size() == static_squares_length.size()) || !(static_squares_x.size() == static_squares_width.size()) ||
            !(static_squares_x.size() == static_squares_orientation.size()))
        {
            ROS_ERROR_STREAM("[Occupancygrid Creator]: Parameters static squares x, y, length, width and orientation dont have the same length!");
            return false;
        }

        for (long unsigned int i=0; i<static_squares_x.size(); i++)
        {
            placeSquareInImage(gridmap_, occupancy_image, static_squares_x[i], static_squares_y[i], static_squares_length[i], static_squares_width[i], static_squares_orientation[i], static_squares_line_thickness[i]);
        }

        placeLineInImage(gridmap_, occupancy_image, 1 , 1, 2, 45, 0.05);
    }



    // Place data back in the gridmap
    gridmap_.data = std::vector<int8_t>(occupancy_image.data, occupancy_image.data + occupancy_image.total());


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
        for (long unsigned int i = 0; i<receiving_obstacle_position_topics.size(); i++)
        {
            subs_vector_[i] = nh_.subscribe<geometry_msgs::PoseStamped>(receiving_obstacle_position_topics[i], 1, std::bind(&OccupancygridCreator::callbackPositionObstacle, this, std::placeholders::_1, i));
        }
    }

    if (receiving_obstacle_position_gazebo_use_)
    {
        sub_gazebo_ = nh_.subscribe(receiving_obstacle_position_gazebo_topic, 1, &OccupancygridCreator::callbackPositionObstacleGazebo, this);
    }

    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &OccupancygridCreator::createMap, this);


    // Show in a window
    //cv::imshow("rectangles", test_image);
    //cv::waitKey(0);
    
    return true;
}

bool OccupancygridCreator::errorFunc(const std::string name)
{
  ROS_ERROR_STREAM("[Occupancygrid Creator]: Parameter \""+name+"\" not defined!");
  return false;
}

void OccupancygridCreator::defaultFunc(const std::string name)
{
    ROS_ERROR_STREAM("[Occupancygrid Creator]: Parameter \""+name+"\" not defined!");
}

void OccupancygridCreator::createMap(const ros::TimerEvent& event)
{
    if (receiving_obstacle_position_use_ && !(std::find(begin(state_received_), end(state_received_), true) == end(state_received_)))
    {
        ROS_WARN_STREAM("[Occupancygrid Creator]: Creating received Obstacles. ");
        // For each of the messages, place in the object
        nav_msgs::OccupancyGrid new_gridmap = gridmap_;
        std::vector<double> x;
        std::vector<double> y;
        for (long unsigned int i=0; i<state_msgs_stored_.size(); i++)
        {
            if (!state_received_[i]){continue;};
            ROS_WARN_STREAM("found :" << state_msgs_stored_[i].pose.position.x);
            double x = state_msgs_stored_[i].pose.position.x;
            double y = state_msgs_stored_[i].pose.position.y;
            placeObstacleInGrid(new_gridmap, x, y, receiving_obstacle_radius_[i]);
        }
        pub_map_.publish(new_gridmap);

        return;
    } else if (receiving_obstacle_position_gazebo_use_ && state_gazebo_received_)
    {
        ROS_WARN_STREAM("[Occupancygrid Creator]: Creating received Gazebo Obstacles. ");
        // For each of the messages, place in the object
        nav_msgs::OccupancyGrid new_gridmap = gridmap_;
        std::vector<double> x;
        std::vector<double> y;
        for (long unsigned int i=0; i<state_msgs_gazebo_stored_.name.size(); i++)
        {
            if (state_msgs_gazebo_stored_.name[i].find("cylinder") == std::string::npos){continue;};
            
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
    for (long unsigned int i=0; i<x.size(); i++)
    {   
        placeObstacleInGrid(gridmap_, x[i], y[i], radius[i]);
    }
}

void OccupancygridCreator::placeObstacleInGrid(nav_msgs::OccupancyGrid &gridmap, double x_cur, double y_cur, double radius_cur)
{
    double rad_steps =  std::atan2(gridmap_.info.resolution, radius_cur);

    for (double radians = 0; radians < 2*3.14159265358979323846; radians = radians + rad_steps)
    {
        double x_on_circle = std::cos(radians)*radius_cur;
        double y_on_circle = std::sin(radians)*radius_cur;

        //convert to map indexes
        int x_on_grid = (x_on_circle+x_cur-gridmap.info.origin.position.x)/gridmap.info.resolution;
        int y_on_grid = (y_on_circle+y_cur-gridmap.info.origin.position.y)/gridmap.info.resolution;

        gridmap.data[x_on_grid+y_on_grid*gridmap_.info.height] = 100;
    }
}

void OccupancygridCreator::placeSquareInImage(nav_msgs::OccupancyGrid &gridmap, cv::Mat &occupancy_image, double x_cur, double y_cur, double length, double width, double orientation, double line_thickness)
{
    int x_on_grid = (x_cur-gridmap.info.origin.position.x)/gridmap.info.resolution;
    int y_on_grid = (y_cur-gridmap.info.origin.position.y)/gridmap.info.resolution;
    int length_on_grid = length/gridmap.info.resolution;
    int width_on_grid = width/gridmap.info.resolution;
    int line_thickness_on_grid = line_thickness/gridmap.info.resolution;

    // first is middle point, then length and width in meters, then orientation in degrees
    cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(x_on_grid, y_on_grid), cv::Size2f(length_on_grid,width_on_grid), orientation);
    cv::Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        cv::line(occupancy_image, vertices[i], vertices[(i+1)%4], cv::Scalar(100), line_thickness_on_grid);
    }
    //cv::Rect brect = rRect.boundingRect();
}

void OccupancygridCreator::placeLineInImage(nav_msgs::OccupancyGrid &gridmap, cv::Mat &occupancy_image, double x_cur, double y_cur, double length, double orientation, double line_thickness)
{
    placeSquareInImage(gridmap, occupancy_image, x_cur, y_cur, length, 0.001, orientation, line_thickness);
}

void OccupancygridCreator::callbackPositionObstacle(const geometry_msgs::PoseStamped::ConstPtr &msg, const long unsigned int i)
{
    state_msgs_stored_[i] = *msg;
    state_received_[i] = true;
}

void OccupancygridCreator::callbackPositionObstacleGazebo(const gazebo_msgs::ModelStates &msg)
{
    state_msgs_gazebo_stored_ = msg;
    state_gazebo_received_ = true;
}
