#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robotpainting/PathPlanningAction.h>

struct LineSegment {
    float startx;
    float starty;

    float endx;
    float endy;
};


class PaintLines{
    public:
        PaintLines(std::vector<LineSegment> segments){  
            ros::NodeHandle nh;
            actionlib::SimpleActionClient<robotpainting::PathPlanningAction> ac("path_planning", true);
            ac.waitForServer();
            ROS_INFO("Server is up.");

            for(const auto& segment : segments){
                std::vector<robotpainting::PathPlanningGoal> goals;
                robotpainting::PathPlanningGoal goal;
                robotpainting::PathPlanningGoal goal1;
                goal.goal.x = segment.startx;
                goal.goal.y = segment.starty;
                goal.isStart = true;

                goal1.goal.x = segment.endx;
                goal1.goal.y = segment.endy;
                goal1.isStart = false;

                goals.push_back(goal);
                goals.push_back(goal1);

                for(robotpainting::PathPlanningGoal goal: goals){
                    ROS_INFO("Sending goal.");
                    ac.sendGoal(goal);

                    bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));

                    if(finished_before_timeout){
                        actionlib::SimpleClientGoalState state = ac.getState();
                        ROS_INFO("Action finished: %s", state.toString().c_str());
                    }else{
                        ROS_INFO("Action did not finish before the time out.");
                    }
                }
                // ROS_INFO("Sending goal.");
                // ac.sendGoal(goal);

                // bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));

                // if(finished_before_timeout){
                //     actionlib::SimpleClientGoalState state = ac.getState();
                //     ROS_INFO("Action finished: %s", state.toString().c_str());
                // }else{
                //     ROS_INFO("Action did not finish before the time out.");
                // }
            }

            // std::vector<robotpainting::PathPlanningGoal> goals;
            // robotpainting::PathPlanningGoal goal1;
            // goal1.goal.x = -2.0;
            // goal1.goal.y = 0.0;
            // goals.push_back(goal1);

            // robotpainting::PathPlanningGoal goal2;
            // goal2.goal.x = -3.0;
            // goal2.goal.y = 0.0;
            // goals.push_back(goal2);

            // robotpainting::PathPlanningGoal goal3;
            // goal3.goal.x = -3.0;
            // goal3.goal.y = 1.0;
            // goals.push_back(goal3);

            // for(robotpainting::PathPlanningGoal goal: goals){
            //     ROS_INFO("Sending goal.");
            //     ac.sendGoal(goal);

            //     bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));

            //     if(finished_before_timeout){
            //         actionlib::SimpleClientGoalState state = ac.getState();
            //         ROS_INFO("Action finished: %s", state.toString().c_str());
            //     }else{
            //         ROS_INFO("Action did not finish before the time out.");
            //     }
            // }
            ROS_INFO("All goals are done.");

        }

    private:
};

std::vector<LineSegment> readLineSegmentsFromFile(const std::string& filename) {
    std::vector<LineSegment> segments;
    std::ifstream file(filename);
    if (file.is_open()) {
        float startx, starty, endx, endy;
        while (file >> startx >> starty >> endx >> endy) {
            segments.push_back({startx, starty, endx, endy});
        }
        file.close();
    } else {
        std::cerr << "Unable to open file for reading\n";
    }
    return segments;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "paintlines");

    std::vector<LineSegment> segments = readLineSegmentsFromFile("/home/mawais/catkin_ws/src/robotpainting/plan/lines.txt");
    // for (const auto& segment : segments) {
    //     std::cout << "startx: " << segment.startx << ", starty: " << segment.starty 
    //               << ", endx: " << segment.endx << ", endy: " << segment.endy << "\n";
    // }

    PaintLines paintlines(segments);

    ros::spin();

    return 0;
}