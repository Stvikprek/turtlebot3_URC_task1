# Aruco Detection

- Aruco marker detection (both in a video and in a stream of images supplied by a ROS node) using the OpenCV library
- The marker dictionary used throughout is the `DICT_6X6_250` dictionary

## Setup
- NOTE: The Aruco Detection provided by OpenCV under its object detection (`objdetect`) module has seen a change in API since OpenCV version 4.7.0. This repo uses a local installation of OpenCV 4.10.0 since the version of OpenCV provided by the `apt` loaded mirrors is version 4.2 (so if you compile the c++ )
- NOTE: Due to the above described reasons and the presence of conflicting libraries, the required `cv_bridge` module (and its parent package `vision_opencv`) have been used as a local installation in the workspace. The main python script `aruco_detect.py` can be found in the `opencv_tests/nodes` folder in the `vision_opencv` directory.

- Install `opencv-contrib-python` (preferably via pip) and uninstall `opencv-python` before to avoid conflicts

## Usage
### Reading a video, detecting Markers and writing to an Output
- If using C++:
```
#include<opencv2/opencv.hpp>
#include<opencv2/objdetect/aruco_detector.hpp>
```
and if using python:
```
import cv2
import cv2.aruco as aruco
```
Most of the python function calls are just the same as the C++ function names and hence the rest of the readme will have C++ snippets only. Simply replace `aruco::FunctionName()` with `aruco.FunctionName()`

- Aruco detection is handled by a `ArucoDetector` object whose constructor should be provided with the parameters being used for Aruco Detection (like min and max marker size/perimeter etc) and the dictionary to which the markers belong to.
```
cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters(); // The parameters in the form of an object
cv::aruco::Dicitonary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250) // Predefined dicitionary of 6X6 Markers 
cv::aruco::ArucoDetector detector(dict,params);
```

- Do the following to read the video file:
```
cv::VideoCapture cap("file.mp4");
// Future code for writing video here
while(1){
    cv::Mat frame;
    cap >> frame;
}
```
This creates a video capture object that reads `file.mp4` and indefinitely stores the current frame in the Mat (matrix) object called the same
- Now, inside the while loop we will perform detection on the frame:
```
while(1){
    cv::Mat frame;
    cap >> frame;
    std::vector<int>marker_ids; // Vector to store the IDs of markers detected
    std::vector<std::vector<Point2f>> corners; // Matrix to store the coordinates of the detected marker corners
    std::vector<std::vector<Point2f>> rejects; // Matrix to store the coordinates of rejected markers, useful for debugging
    detector.detectMarkers(frame,corners,marker_ids,rejects); // Populates the vectors with data extracted from the frame
}

```
- To write our results to an `output` file, we will use the `cv::VideoWriter` object. We must instantiate this object before the while loop and write each output frame into the file. The constructor's arguments are the following: output filename, the fourcc code (ASCII code to identify codecs),fps of the output and its size/resolution
```
cv::VideoWriter out_write("output.mp4",cv::VideoWriter::fourcc('M','P','E','G'),30,Size(width,height));
```
where the width and height of the frame capture can be easily obtained using `cap.get(cv::CAP_PROP_FRAME_WIDTH)` and `cap.get(cv::CAP_PROP_FRAME_HEIGHT)`

- Now in the while loop: 
```
while(1){
    // Detetcion code here
    cv::Mat output_frame = frame.clone(); // Copy the captured frame
    cv::aruco::drawDetectedMarkers(output_frame,corners,marker_ids); // Draw the detected markers onto the copied frame
    out_write.write(output_frame);
}

```

You can also set a proper condition inside the while's conditional, something like `cap.read(frame) != NULL` or exiting once a certain key is pressed.

### Aruco Marker Detection in a Gazebo simulated world (using Python)
- While the concept roughly remains the same, and the detection code too, this time we will be obtaining input information from a ROS topic like `/camera/rgb/image_raw` and since ROS communicates images in the form of a sensor message, we need to convert it into a form usable by OpenCV using the `cv_bridge` module
- First, import everything necessary:
```
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
```
- Now instantiate the `CvBridge` object along with the aruco detector
```
br = CvBridge()
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)
```

- Next we need a basic ROS node to subscribe to a topic onto which the robot's visual sensor data is being published:
```
if __name__ == '__main__':   
    rospy.init_node('aruco_detect')
    rospy.Subscriber('/camera/rgb/image_raw', sensor_msgs.msg.Image, detect)
    rospy.spin()
```
whose callback function is called `detect` which looks like:
```
def detect(imgmsg):
    img = br.imgmsg_to_cv2(imgmsg, "bgr8") #Convert the imgmsg to cv2 readable form,using the provided pixel color format
    corns,ids,rejects = detector.detectMarkers(img) #Perform detection and populate the arrays just like before
    print(ids) # Here you can also draw onto the image (using previous methods) and publish it to some other ROS topic

```
