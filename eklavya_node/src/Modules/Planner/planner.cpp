#include "planner.h"
#include "plannerMethods.h"

using namespace cv;
namespace planner_space {


    /// ------------------------------------------------------------- ///

    	void  Planner::loadPlanner() {
        loadSeeds();
        ROS_INFO("[PLANNER] Seeds Loaded");

        initBot();
        ROS_INFO("[PLANNER] Vehicle Initiated");
    }

    	geometry_msgs::Twist  Planner::findPath(Triplet bot, Triplet target,Mat map_img) {
       
       geometry_msgs::Twist cmdvel;
        state start, goal;
        start.pose = bot;
        start.seed_id = -1;
        start.g = 0;
        start.h = distance(bot, target);
        goal.pose = target;
        goal.g = 0;
        goal.h = 0;
        goal.seed_id = 0;
        brake.vl = brake.vr = 0;

        vector<state> open_list;
        open_list.insert(open_list.begin(), start);
        map<Triplet, open_map_element, PoseCompare> open_map;
        open_map[start.pose].membership = OPEN;
        open_map[start.pose].cost = start.g;

        map<Triplet, state, PoseCompare> came_from;
        
        ros::NodeHandle nh;
        ros::Publisher target_pub = nh.advertise<std_msgs::String>("target_reached", 20);
        std_msgs::String msg;
        msg.data = "REACHED YO YO";

        if (isEqual(start, goal)) {
            ROS_INFO("[PLANNER] Target Reached");
            //Planner::finBot();
            target_pub.publish(msg);
            return cmdvel;
        }

        int iterations = 0;
        while (!open_list.empty()) {
            //TODO: This condition needs to be handled in the strategy module.
            if (local_map[start.pose.x][start.pose.y] > 0) {
                ROS_WARN("[PLANNER] Robot is in Obstacles");
                Planner::finBot();
                return cmdvel;
            }

            state current = open_list.front();
#ifdef DEBUG
            plotPoint(map_img,current.pose);
          cv::imshow("[PLANNER] Map", map_img);
                cvWaitKey(0);
#endif

            if ((open_map.find(current.pose) != open_map.end()) &&
                    (open_map[current.pose].membership == CLOSED)) {
                continue;
            }

            if (isEqual(current, goal)) {

               cmdvel=reconstructPath(came_from, current,map_img);

#ifdef DEBUG
                ROS_INFO("[PLANNER] Path Found");

                cv::imshow("[PLANNER] Map", map_img);
                cvWaitKey(0);
#endif

                resize(map_img, showImage1,cvSize(400,400));
                cv::imshow("[PLANNER] Map", showImage1);
                cvWaitKey(WAIT_TIME);
                closePlanner();
                
                return cmdvel;
            }

             if(onTarget(current ,goal))
             {

               cmdvel=reconstructPath(came_from, current,map_img);

#ifdef DEBUG
                ROS_INFO("[PLANNER] Path Found");
                cv::imshow("[PLANNER] Map", map_img);   
                cvWaitKey(0);
#endif
                resize(map_img,showImage2,cvSize(400,400));
                 cv::imshow("[PLANNER] Map", showImage2);
                cvWaitKey(WAIT_TIME);
                closePlanner();
                
                return cmdvel;

             }

            //TODO : why pop_heap 
            pop_heap(open_list.begin(), open_list.end(), StateCompare());
            open_list.pop_back();
            //open_map[current.pose].membership = UNASSIGNED;
            open_map[current.pose].cost = -1;

            open_map[current.pose].membership = CLOSED;

            vector<state> neighbors = neighborNodes(current);

            for (unsigned int i = 0; i < neighbors.size(); i++) {
                state neighbor = neighbors[i];

#ifdef DEBUG
             
                //plotPoint(map_img,neighbor.pose);
#endif

                if (!(((neighbor.pose.x >= 0) && (neighbor.pose.x < MAP_MAX)) &&
                        ((neighbor.pose.y >= 0) && (neighbor.pose.y < MAP_MAX)))) {
                    continue;
                }

                if (!isWalkable(current, neighbor)) {
                    continue;
                }

                double tentative_g_score = neighbor.g + current.g;
                double admissible = distance(neighbor.pose, goal.pose);
                double consistent = admissible;
                //please explain next condition 
                //@TODO :if already in open_list update cost and came_from if cost is less
                if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                        (open_map[neighbor.pose].membership == OPEN))) {
                    came_from[neighbor.pose] = current;
                    neighbor.g = tentative_g_score;
                    neighbor.h = consistent;
                    //next condition is always true
                    if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                            (open_map[neighbor.pose].membership == OPEN))) {
                        open_list.push_back(neighbor);
                        push_heap(open_list.begin(), open_list.end(), StateCompare());
                        open_map[neighbor.pose].membership = OPEN;
                        open_map[neighbor.pose].cost = neighbor.g;
                    }
                }
            }

            iterations++;
            if (iterations > MAX_ITER) {
                ROS_WARN("[PLANNER] Open List Overflow");
                Planner::finBot();
                return cmdvel;
            }
        }

        ROS_ERROR("[PLANNER] No Path Found");
        closePlanner();
        Planner::finBot();
        return cmdvel;
    }

    void Planner::finBot() {
        sendCommand(brake);
    }
}
