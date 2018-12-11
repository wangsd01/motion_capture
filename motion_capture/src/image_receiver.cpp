#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_calibration_parsers/parse.h>
#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif
#include <iostream>

using namespace cv;
using namespace std;

cv::VideoWriter outputVideo;

int g_count = 0;
ros::Time g_last_wrote_time = ros::Time(0);
std::string encoding;
std::string codec;
int fps;
std::string filename;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg)->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int image_process() {
  cv::Mat image;
  std::string image_folder("/home/wang/catkin_ws/src/motion_capture/src/");
  std::string image_name(image_folder+"rope_image.png");
  image = cv::imread(image_name, CV_LOAD_IMAGE_COLOR);
  if(! image.data )                              // Check for invalid input
  {
      std::cout <<  "Could not open or find the image" << std::endl ;
      return -1;
  }

  Mat gray_image;
  cvtColor( image, gray_image, CV_BGR2GRAY );
  imwrite(image_folder+"/gray_rope_image.png", gray_image);

  for(int i=0; i < gray_image.rows; i++) {
    for(int j=0; j < gray_image.cols; j++) {
      Scalar intensity = gray_image.at<uchar>(i, j);

      if(intensity.val[0] > 50) {
        gray_image.at<uchar>(i, j) = 255;
      }
      if(i==0 && j==0) {
        cout << intensity << std::endl;
        cout << gray_image.at<uchar>(i, j) << std::endl;
      }
    }
  }
  imwrite(image_folder+"/modified_gray_rope_image.png", gray_image);

  cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  cv::imshow( "Display window", image );   
  cv::waitKey(0);
  return 0;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "image_receiver");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  cv::destroyWindow("view");



  return 0;
}