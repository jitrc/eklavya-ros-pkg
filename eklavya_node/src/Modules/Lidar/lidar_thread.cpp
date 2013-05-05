#include "LidarData.h"

//#define FPS_TEST

char **laser_scan;
IplImage* showImg1;
IplImage* showImg2;
IplImage* showImg3;


void *lidar_thread(void *arg) {
    ros::NodeHandle lidar_node;

    showImg1=cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);
    showImg2=cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);
    showImg3=cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);

    ros::Subscriber sub = lidar_node.subscribe("scan", 2, LidarData::update_map);
    ros::Rate loop_rate(LOOP_RATE);

    ROS_INFO("Lidar thread started");

#ifdef FPS_TEST
    ROS_INFO("[LIDAR] Conducting an FPS Test");
    int iterations = 0;
    time_t start = time(0);
#endif

    while (ros::ok()) {
#ifdef FPS_TEST
        if (iterations > 100) {
            time_t finish = time(0);
            double fps = (iterations + 0.0) / (finish - start);
            ROS_INFO("[LIDAR] FPS: %lf", fps);
            break;
        }

        iterations++;
#endif

        ros::spinOnce();
        loop_rate.sleep();
    }

    return NULL;
}

