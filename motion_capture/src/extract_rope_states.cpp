#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int main(int, char**)
{
    // string filename = "/home/wang/catkin_ws/depth.avi";
    string filename = "/home/wang/catkin_ws/color.avi";
    VideoCapture cap(filename);
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("image",1);
    bool save_first = false;
    for(int i=0;;i++)
    {
        Mat frame;
        cap >> frame; // get a new frame from file
        if(frame.empty())
            break;
        // cvtColor(frame, edges, COLOR_BGR2GRAY);
        // GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        // Canny(edges, edges, 0, 30, 3);
        imshow("image", frame);
        if(save_first) {
            imwrite("/home/wang/catkin_ws/color_images/color_image.png", frame);
            save_first = false;
            break;
        } else {
            imwrite("/home/wang/catkin_ws/color_images/color_image_" + std::to_string(i) + ".png", frame);
        }
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}