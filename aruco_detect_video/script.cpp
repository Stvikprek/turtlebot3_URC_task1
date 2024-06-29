#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

int main(){

    VideoCapture capture("test.mp4");
    int frame_width = capture.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    VideoWriter vid("output.avi",cv::VideoWriter::fourcc('M','J','P','G'), 30, Size(frame_width,frame_height)); 

    aruco::DetectorParameters params = aruco::DetectorParameters();
    aruco::Dictionary dict = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    aruco::ArucoDetector det(dict,params);

    while(1){
        Mat img;
        capture >> img;
        std::vector<int> markerid;
        std::vector<std::vector<Point2f>> corners,rejections;
        det.detectMarkers(img,corners,markerid,rejections);
        Mat output = img.clone();
        aruco::drawDetectedMarkers(output,corners,markerid);
        vid.write(output);
        std::cout << markerid.empty() << " " << rejections.empty() << " " << corners.empty() << std::endl;

    }

}