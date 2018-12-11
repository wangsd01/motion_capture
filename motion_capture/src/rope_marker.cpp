#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    Mat image;
    image = imread("/home/wang/catkin_ws/color_image.png", CV_LOAD_IMAGE_COLOR);
    // namedWindow("Display color image", WINDOW_AUTOSIZE);

    for (int i=0; i < image.rows; i++) {
        for (int j=0; j < image.cols; j++) {
            Vec3b& intensity = image.at<Vec3b>(i, j);
            if(intensity.val[0]>=120 && intensity.val[0]<=225) { // B
                if (intensity.val[1]>= 56 && intensity.val[1]<=130) { // G
                    if (intensity.val[2]>=108 && intensity.val[2]<= 190) { // R
                        intensity.val[0] = 255;
                        intensity.val[1] = 0;
                        intensity.val[2] = 0;
                        std::cout << "mark pixel " << i << " " << j << " as blue!" << std::endl;
                    }
                }
            }

        }
    }

    imshow("Display window", image);

    waitKey(0);
    return 0;
}