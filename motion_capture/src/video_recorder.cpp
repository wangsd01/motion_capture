/****************************************************************************
* Software License Agreement (Apache License)
*
*     Copyright (C) 2012-2013 Open Source Robotics Foundation
*
*     Licensed under the Apache License, Version 2.0 (the "License");
*     you may not use this file except in compliance with the License.
*     You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
*     Unless required by applicable law or agreed to in writing, software
*     distributed under the License is distributed on an "AS IS" BASIS,
*     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*     See the License for the specific language governing permissions and
*     limitations under the License.
*
*****************************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif

namespace enc = sensor_msgs::image_encodings;

int g_count = 0;
std::string encoding;
std::string codec;
int fps;
std::string filename;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;

void callback(const sensor_msgs::ImageConstPtr &image_msg, cv::VideoWriter &outputVideo, ros::Time &last_wrote_time, std::string encoding, std::string filename)
{
    if (!outputVideo.isOpened())
    {

        cv::Size size(image_msg->width, image_msg->height);

        outputVideo.open(filename,
#if CV_MAJOR_VERSION == 3
                         cv::VideoWriter::fourcc(codec.c_str()[0],
#else
                         CV_FOURCC(codec.c_str()[0],
#endif
                                                 codec.c_str()[1],
                                                 codec.c_str()[2],
                                                 codec.c_str()[3]),
                         fps,
                         size,
                         true);

        if (!outputVideo.isOpened())
        {
            ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
            exit(-1);
        }

        ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording.");
    }

    if ((image_msg->header.stamp - last_wrote_time) < ros::Duration(1 / fps))
    {
        // Skip to get video with correct fps
        return;
    }

    try
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        if (enc::isColor(image_msg->encoding))
        {
            std::cout << "color image got!" << std::endl;
            cv_bridge::CvtColorForDisplayOptions options;
            options.do_dynamic_scaling = use_dynamic_range;
            options.min_image_value = min_depth_range;
            options.max_image_value = max_depth_range;
            options.colormap = colormap;
            cv_ptr = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding, options);
        }
        else
        {
            std::cout << "depth image got!" << std::endl;
            cv_ptr = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg));
        }

        const cv::Mat image = cv_ptr->image;
        if (!image.empty())
        {
            outputVideo << image;
            ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
            g_count++;
            last_wrote_time = image_msg->header.stamp;
        }
        else
        {
            ROS_WARN("Frame skipped, no data!");
        }
    }
    catch (cv_bridge::Exception)
    {
        ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    local_nh.param("filename", filename, std::string("output.avi"));
    bool stamped_filename;
    local_nh.param("stamped_filename", stamped_filename, false);
    local_nh.param("fps", fps, 15);
    local_nh.param("codec", codec, std::string("MJPG"));
    local_nh.param("encoding", encoding, std::string("bgr8"));
    // cv_bridge::CvtColorForDisplayOptions
    local_nh.param("min_depth_range", min_depth_range, 0.0);
    local_nh.param("max_depth_range", max_depth_range, 0.0);
    local_nh.param("use_dynamic_depth_range", use_dynamic_range, false);
    local_nh.param("colormap", colormap, -1);

    if (stamped_filename)
    {
        std::size_t found = filename.find_last_of("/\\");
        std::string path = filename.substr(0, found + 1);
        std::string basename = filename.substr(found + 1);
        std::stringstream ss;
        ss << ros::Time::now().toNSec() << basename;
        filename = path + ss.str();
        ROS_INFO("Video recording to %s", filename.c_str());
    }

    if (codec.size() != 4)
    {
        ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

    image_transport::ImageTransport it(nh);

    std::string color_img_topic = nh.resolveName("/camera/color/image_raw");
    cv::VideoWriter output_color_video;
    ros::Time color_last_wrote_time = ros::Time(0);
    std::string color_encoding("bgr8");
    image_transport::Subscriber sub_color_image = it.subscribe(color_img_topic, 1, boost::bind(callback, _1, output_color_video, color_last_wrote_time, color_encoding, "color.avi"));
    ROS_INFO_STREAM("Waiting for topic " << color_img_topic << "...");

    std::string depth_img_topic = nh.resolveName("/camera/aligned_depth_to_color/image_raw");
    cv::VideoWriter output_depth_video;
    ros::Time depth_last_wrote_time = ros::Time(0);
    std::string depth_encoding("");
    image_transport::Subscriber sub_depth_image = it.subscribe(depth_img_topic, 1, boost::bind(callback, _1, output_depth_video, depth_last_wrote_time, depth_encoding, "depth.avi"));
    ROS_INFO_STREAM("Waiting for topic " << depth_img_topic << "...");
    ros::spin();
    std::cout << "\nStopped!" << std::endl;
}