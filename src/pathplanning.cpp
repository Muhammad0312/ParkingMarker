#include <string>
#include <vector>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <robotpainting/PathPlanningAction.h>
#include <geometry_msgs/Point.h>


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/config.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;

class PathPlanning
{
    public:
    PathPlanning(const std::string &mapTopic){
        // Node Handle
        ros::NodeHandle nh;
        space = std::make_shared<ob::SE2StateSpace>();
        
        // intialize the action server
        as_ = new actionlib::SimpleActionServer<robotpainting::PathPlanningAction>(nh, action_name_, boost::bind(&PathPlanning::planCB, this, _1), false);
        as_->start();

        // Save current time
        lastTime = ros::Time::now();
        // Allocate size for position
        currentPosition.resize(3);
        goal.resize(2);
        origin.resize(3);
        mapPositionCurrent.resize(3);
        mapPositionGoal.resize(3);
        path_points.resize(0);

        mapSub = nh.subscribe(mapTopic, 10, &PathPlanning::mapCallback, this);
        odomSub = nh.subscribe("/odom", 10, &PathPlanning::odomCallback, this);
        goalSub = nh.subscribe("/clicked_point", 10, &PathPlanning::goalCallback, this);

        pathPub = nh.advertise<nav_msgs::Path>("/path", 10);
        occupancyPub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_stored", 10);
        markerPub = nh.advertise<visualization_msgs::MarkerArray>("/marker", 10);
        cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        linePub = nh.advertise<geometry_msgs::PointStamped>("/line", 10);

        // Call the controller at using timer every 0.1s
        timer = nh.createTimer(ros::Duration(0.1), &PathPlanning::controller, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapData){
        if((mapData->header.stamp - lastTime).toSec() > 1){
            lastTime = mapData->header.stamp;
            resolution = mapData->info.resolution;
            origin[0] = mapData->info.origin.position.x;
            origin[1] = mapData->info.origin.position.y;
            origin[2] = mapData->info.origin.position.z;

            numBlocks = std::round(radius / resolution);
            int width = mapData->info.width;
            int height = mapData->info.height;
            occupancyMap.clear();
            occupancyMap.resize(height, std::vector<int8_t>(width));
            
            for (size_t i = 0; i < height; i++)
            {
                for (size_t j = 0; j < width; j++)
                {
                    occupancyMap[i][j] = mapData->data[i * width + j];
                }
            }
            occupancyGrid();
            if(path_points.size() > 0){
                if(checkPath()){
                    // ROS_INFO("Path is valid");
                }
                else{
                    ROS_INFO("Path is invalid");
                    ROS_INFO("Replanning!!!");
                    path_points.clear();
                    geometry_msgs::Twist cmdVel;
                    cmdVel.linear.x = 0;
                    cmdVel.angular.z = 0;
                    cmdVelPub.publish(cmdVel);
                    plan(currentPosition, goal);
                }
            }
        }     
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odomData){
        currentPosition[0] = odomData->pose.pose.position.x;
        currentPosition[1] = odomData->pose.pose.position.y;
        qx = odomData->pose.pose.orientation.x;
        qy = odomData->pose.pose.orientation.y;
        qz = odomData->pose.pose.orientation.z;
        qw = odomData->pose.pose.orientation.w;
        currentPosition[2] = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        positionToMap(currentPosition, mapPositionCurrent);
    }

    void goalCallback(const geometry_msgs::PointStamped::ConstPtr& goalData){
        goal[0] = goalData->point.x;
        goal[1] = goalData->point.y;
        positionToMap(goal, mapPositionGoal);
        // check if current position is valid
        isStateValid(currentPosition);
        plan(currentPosition, goal);
        
    }

    void positionToMap(std::vector<float>& pos, std::vector<int>& map){
        // Convert current position to map coordinates i.e. cells
        map[0] = std::round((pos[0] - origin[0]) / resolution);
        map[1] = std::round((pos[1] - origin[1]) / resolution);
    }

    bool isStateValid(const ob::State *state)
    {
        //Check if the state is valid
        const auto *real_vector_state = state->as<ob::SE2StateSpace::StateType>();
        float x = real_vector_state->getX();
        float y = real_vector_state->getY();

        std::vector<int> mapPosition;
        mapPosition.resize(2);
        std::vector<float> xy = {x, y};
        positionToMap(xy, mapPosition);

        for (int i = -numBlocks - 1; i < numBlocks; i++)
        {
            for (int j = -numBlocks; j < numBlocks + 1; j++)
            {
                x = mapPosition[0] + i;
                y = mapPosition[1] + j;

                if(x < 0 || x >= occupancyMap[0].size() || y < 0 || y >= occupancyMap.size()){
                    return false;
                }
                else if(occupancyMap[y][x] > occupancyThreshold){
                    return false;
                }

                if(lineEnded){
                    for (const auto &line : lines)
                    {
                        if (x == line[0] && y == line[1])
                        {
                            return false;
                        }
                    }
                }
            }
        }
        
        return true;
    }

    bool isStateValid(std::vector<float> &pos)
    {
        std::vector<int> mapPosition;
        mapPosition.resize(2);
        positionToMap(pos, mapPosition);

        // Create a marker array
        visualization_msgs::MarkerArray marker_array;
        int num = 0;

        for (int i = -numBlocks - 1; i < numBlocks; i++)
        {
            for (int j = -numBlocks; j < numBlocks+1; j++)
            {
                int x = mapPosition[0] + i;
                int y = mapPosition[1] + j;

                // Publish marker at this point
                std::vector<float> point;
                point.resize(2);

                // Convert to position
                point[0] = x * resolution + origin[0];
                point[1] = y * resolution + origin[1];

                visualization_msgs::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = ros::Time();
                marker.ns = "points";
                marker.id = num++;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = float(point[0]);
                marker.pose.position.y = float(point[1]);
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.02;
                marker.scale.y = 0.02;
                marker.scale.z = 0.02;

                // Set a different color for each marker
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0; //1.0 - (float)i / num
                marker.color.a = 1.0;

                if(occupancyMap[y][x] < occupancyThreshold){
                    marker_array.markers.push_back(marker);
                }
            }
        }
        markerPub.publish(marker_array);
        
        return true;
    }


    void plan(std::vector<float> &current, std::vector<float> &end)
    {
        // construct the state space we are planning in
        // auto space(std::make_shared<ob::SE2StateSpace>());

        // set the bounds for the R^2 part of SE(2)
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, -10);  // x lower bound
        bounds.setHigh(0, 10);  // x upper bound
        bounds.setLow(1, -10);  // y lower bound
        bounds.setHigh(1, 10);  // y upper bound

        space->setBounds(bounds);
        space->setLongestValidSegmentFraction(0.001);

        // construct an instance of  space information from this state space
        auto si(std::make_shared<ob::SpaceInformation>(space));

        // set state validity checking for this space
        si->setStateValidityChecker([this](const ompl::base::State *state) { return isStateValid(state); });

        // create a start state
        ob::ScopedState<> start(space);
        start[0] = current[0];
        start[1] = current[1];
        // print the start state
        std::cout << "Start: ";
        start.print(std::cout);

        // create a goal state
        ob::ScopedState<> goal(space);
        goal[0] = end[0];
        goal[1] = end[1];
        std::cout << "Goal: ";
        goal.print(std::cout);

        // create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        // create a planner for the defined space
        // auto planner(std::make_shared<og::RRTConnect>(si));
        auto planner(std::make_shared<og::RRTstar>(si));

        // set the problem we are trying to solve for the planner
        planner->setProblemDefinition(pdef);

        // perform setup steps for the planner
        planner->setup();


        // print the settings for this space
        // si->printSettings(std::cout);

        // print the problem settings
        // pdef->print(std::cout);

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);

        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            ob::PathPtr path = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;

            // print the path to screen
            // path->print(std::cout);

            path_points.clear();

            // Cast to PathGeometric to extract states
            const og::PathGeometric &path_geometric = static_cast<const og::PathGeometric &>(*path);

            // Store states in a vector
            for (std::size_t i = 0; i < path_geometric.getStateCount(); ++i)
            {
                const ob::SE2StateSpace::StateType *state = path_geometric.getState(i)->as<ob::SE2StateSpace::StateType>();
                std::vector<double> point = {state->getX(), state->getY(), state->getYaw()};
                path_points.push_back(point);
            }

            // Print the states
            for (const auto &point : path_points)
            {
                std::cout << "Point: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
            }

            publishPath(path_points);
        }
        else
            std::cout << "No solution found" << std::endl;
    }

    void planCB(const robotpainting::PathPlanningGoalConstPtr &goal)
    {   
        current_goal_ = goal;
        std::vector<float> goalPosition = {float(goal->goal.x), float(goal->goal.y)};

        plan(currentPosition, goalPosition);
        // Wait for the goal to be reached
        while(path_points.size() > 0){
            if(as_->isPreemptRequested() || !ros::ok()){
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_->setPreempted();
                break;
            }
        }

        if(goal->isStart){
            ROS_INFO("Start Point");
            startx = goal->goal.x;
            starty = goal->goal.y;
            lineStarted = true;
        }
        else{
            endx = goal->goal.x;
            endy = goal->goal.y;
            lineStarted = false;
            float diff = 0;
            if(startx > endx){
                diff = -0.4;
            }
            else{
                diff = 0.4;
            }
            ROS_INFO("End Point, Moving a bit forward: %f", diff);
            std::vector<double> goalPoint = {double(currentPosition[0] + diff), double(currentPosition[1])};
            path_points.push_back(goalPoint);
            // std::vector<float> goalPoint = {currentPosition[0] + diff, currentPosition[1]};
            // plan(currentPosition, goalPoint);
        }

        // Wait for the goal to be reached
        while(path_points.size() > 0){
            if(as_->isPreemptRequested() || !ros::ok()){
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_->setPreempted();
                break;
            }
        }

        if(goal->isStart){
            lineEnded = false;
        }
        else{
            lineEnded = true;
        }

        if(path_points.size() == 0){
            as_->setSucceeded();
        }
    }

    void publishPath(std::vector<std::vector<double>> &path_points)
    {
        nav_msgs::Path path;
        path.header.frame_id = "odom";
        path.header.stamp = ros::Time::now();

        for (const auto &point : path_points)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point[0];
            pose.pose.position.y = point[1];
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            path.poses.push_back(pose);
        }

        pathPub.publish(path);
    }

    void occupancyGrid(){
        nav_msgs::OccupancyGrid occupancy;
        occupancy.header.frame_id = "odom";
        occupancy.header.stamp = ros::Time::now();
        occupancy.info.resolution = resolution;
        occupancy.info.width = occupancyMap[0].size();
        occupancy.info.height = occupancyMap.size();
        occupancy.info.origin.position.x = origin[0];
        occupancy.info.origin.position.y = origin[1];
        occupancy.info.origin.position.z = origin[2];

        for (size_t i = 0; i < occupancyMap.size(); i++)
        {
            for (size_t j = 0; j < occupancyMap[0].size(); j++)
            {
                occupancy.data.push_back(occupancyMap[i][j]);
            }
        }

        occupancyPub.publish(occupancy);
    }

    void controller(const ros::TimerEvent& event){
        geometry_msgs::Twist cmdVel;
        cmdVel.linear.x = 0.0;
        cmdVel.angular.z = 0.0;

        // Implement a controller to follow the path
        if(path_points.size() > 0){
            // Get the first point in the path
            std::vector<double> point = path_points[0];
            // Calculate the angle to the point
            double angle = atan2(point[1] - currentPosition[1], point[0] - currentPosition[0]);
            angle = angle - currentPosition[2];
            wrapAngleToPi(angle);

            // Calculate the angular velocity
            double angular = kOmega * angle;
            // Calculate the linear velocity
            double linear = kV * sqrt(pow(point[0] - currentPosition[0], 2) + pow(point[1] - currentPosition[1], 2));

            // Check if the robot is close to the point
            if(sqrt(pow(currentPosition[0] - point[0], 2) + pow(currentPosition[1] - point[1], 2)) < 0.01
            ){
                // Remove the point at index 0
                path_points.erase(path_points.begin());
            }

            if(lineStarted){
                angleThreshold = 0.01;
                kOmega = 0.3;
                maxLinear = 0.1;
                std::vector<float> point = {float(currentPosition[0]), float(currentPosition[1])};
                publishMarker(point);
                std::vector<int> mapPosition;
                mapPosition.resize(2);
                positionToMap(point, mapPosition);
                lines.push_back(mapPosition);
                // Remove duplicates
                lines.erase(std::unique(lines.begin(), lines.end()), lines.end());
            }
            else{
                angleThreshold = 0.1;
                kOmega = 0.5;
                maxLinear = 0.2;
            }
            if(path_points.size() == 0){
                ROS_INFO("Goal Reached");
            }
            // Publish the velocity command
            if(angle > angleThreshold || angle < -angleThreshold){
                linear = 0;
            }
            
            // Publish the velocity command
            // Clip velocity
            if(linear > maxLinear){
                linear = maxLinear;
            }
            else if(linear < -maxLinear){
                linear = -maxLinear;
            }
            cmdVel.linear.x = linear;
            cmdVel.angular.z = angular;
        }
        // Move the robot towards the point
        cmdVelPub.publish(cmdVel);
    }

    void wrapAngleToPi(double &angle) {
        angle =  angle - 2 * M_PI * std::floor((angle + M_PI) / (2 * M_PI));
    }

    bool checkPath(){
        // Descritize the path and check if any obstacles are in the path
        float step = 0.1;
        // Check if path from current position to next point is valid
        std::vector<double> point1 = path_points[0];
        float distance = sqrt(pow(point1[0] - currentPosition[0], 2) + pow(point1[1] - currentPosition[1], 2));
        int numSteps = distance / step;

        for (int j = 0; j < numSteps; j++){
            float x = currentPosition[0] + j * (point1[0] - currentPosition[0]) / numSteps;
            float y = currentPosition[1] + j * (point1[1] - currentPosition[1]) / numSteps;
            
            ob::ScopedState<ob::SE2StateSpace> testState(space); 
            testState->setXY(x, y);
            if(!isStateValid(testState.get())){
                return false;
            }
        }
        return true;
    }

    void publishMarker(std::vector<float> &point){
        geometry_msgs::PointStamped marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.point.x = point[0];
        marker.point.y = point[1];
        marker.point.z = 0;

        linePub.publish(marker);
    }

    private:

    // Subscribers
    ros::Subscriber mapSub;
    ros::Subscriber odomSub;
    ros::Subscriber goalSub;

    // Publishers
    ros::Publisher pathPub;
    ros::Publisher occupancyPub;
    ros::Publisher markerPub;
    ros::Publisher cmdVelPub;
    ros::Publisher linePub;

    // Subscribed data
    // Map Data
    ros::Time lastTime;
    std::vector<std::vector<int8_t>> occupancyMap;
    float resolution;
    std::vector<float> origin;
    int occupancyThreshold = 30;

    // Odom Data
    std::vector<float> currentPosition;
    double qx, qy, qz, qw;
    std::vector<int> mapPositionCurrent;

    // Goal Data
    std::vector<float> goal;
    std::vector<int> mapPositionGoal;

    // Robot Data
    float radius = 0.20;
    int numBlocks;

    // Path
    std::vector<std::vector<double>> path_points;

    // Timer
    ros::Timer timer;

    // OMPL
    std::shared_ptr<ob::SE2StateSpace> space;

    // Action Server
    actionlib::SimpleActionServer<robotpainting::PathPlanningAction>* as_;
    std::string action_name_ = "path_planning";
    robotpainting::PathPlanningFeedback feedback_;
    robotpainting::PathPlanningResult result_;
    robotpainting::PathPlanningGoalConstPtr current_goal_;

    // Line data
    int startx = 0;
    int starty = 0;
    int endx = 0;
    int endy = 0;

    bool lineStarted = false;
    bool lineEnded = false;
    std::vector<std::vector<int>> lines;

    // Controller Data
    float angleThreshold = 0.1;
    float kV = 0.5;
    float kOmega = 0.5;
    float maxLinear = 0.2;
    float maxAngular = 0.5;

};


int main(int argc, char** argv){

    ros::init(argc, argv, "pathplanning");

    PathPlanning pp("/projected_map");

    ros::spin();

    return 0;
}