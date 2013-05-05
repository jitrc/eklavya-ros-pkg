#include "lane_data.h"

 IplImage *show_img1;
 IplImage *show_img2;
 IplImage *show_img3;
 IplImage *show_img4;
void *lane_thread(void *arg) {
  ros::NodeHandle lane_node;
  show_img1=cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);;
  show_img2=cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);;
  show_img3=cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);;
  show_img4=cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 3);;

  LaneDetection lane_d;
  image_transport::ImageTransport it(lane_node);
  image_transport::Subscriber sub = it.subscribe("camera/image", 2, &LaneDetection::markLane, &lane_d);
  ros::Rate loop_rate(LOOP_RATE);
  ROS_INFO("LANE_THREAD STARTED");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Lane code exiting");
  
  return NULL;
}
