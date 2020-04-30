/**
 * @file realsense_apriltag.cpp
 * @author: Megan Reinhart
 *
 * Code based off open source "Getting Started with OpenCV" with librealsense on GitHub and 
 * an open source AprilTags C++ library available on Linux and Mac. By using librealsense C++
 * as a high level API library, streaming from the RealSense is simplified by a few hundred lines
 * of code while adding addtional features. These include modifying the reference point to an 
 * selected AprilTag and adding checks to this parameter. The output is carriage returned and
 * cleared in the terminal as it continously updates in a pop up window using OpenCV. Smoothing 
 * was added for the continous updates as every so often a tag wouldn't be recognized in a frame. 
 * By using the librealsense API, additonal use of pointcloud referencing and checks would be simple 
 * to impliment and layer, but have yet to be included.
 */

#include <cmath>
#include <map>
#include <sstream>
#include <string>

#include <librealsense2/rs.hpp> // Include the librealsense C++ header file
#include <opencv2/opencv.hpp> // Include OpenCV header file

// AprilTags detector and tag types
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

using namespace std;

// Normalizes angle to be within the interval [-pi,pi]
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

// Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

const int width = 848; // RealSense image size in pixels
const int height = 480;
const double tagSize = 0.07; // Input the side length of the tags in meters
const double fx = 600; // Focal length for RealSense D435i is 1.93 mm, input pixels
const double fy = 600; 
const double px = width / 2; // RealSense principal points, or center of camera in pixels
const double py = height / 2;

string getDetectionString(AprilTags::TagDetection & detection, Eigen::Vector3d & translation, Eigen::Matrix3d & rotation){

    detection.getRelativeTranslationRotation(tagSize, fx, fy, px, py, translation, rotation); // Based on TagDetection.h

    Eigen::Matrix3d F;
    F <<
    1, 0,  0,
    0,  -1,  0,
    0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    stringstream ss;

    // Currently not including yaw, pitch, roll in print out based on the selected tag reference
    ss << "Id: " << detection.id
        << ", distance=" << translation.norm()
        << "m, x=" << translation(0)
        << ", y=" << translation(1)
        << ", z=" << translation(2)
        << ", yaw=" << yaw
        << ", pitch=" << pitch
        << ", roll=" << roll;

    return ss.str();
}

int main()
{
    rs2::pipeline pipe; // Contruct a pipeline which abstracts the device
    rs2::config cfg; // Create a configuration for configuring the pipeline with a non default profile

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);

    // Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg); 

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++){
        frames = pipe.wait_for_frames(); // Wait for all configured streams to produce a frame
    }

    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE); // Create a window in OpenCV

    int smoothingFrames = 20; // Parameter for frames skipped in smoothing, play around with it
    int smoothingDisplay = 10; // How many milliseconds before the AprilTag is checked for being in frame
    int lastCount = 0; // Count for display detection

    int referenceTagId = 2; // What tag are we moving the origin to? Use AprilTag ID
    
    map<int, int> lru; // Least recently used - this keeps track of how long it's been since we've seen a tag
    map<int, string> outputStrings; 
    map<int, Eigen::Vector3d> vectorsFromCamera;

    while (true) {

        frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame(); //Get each frame

        // Creating OpenCV Matrix from a color image
        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat image_gray(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        
        // AprilTags can only be read in grayscale, so convert color image
        cv::cvtColor(image, image_gray, CV_BGR2GRAY);
        AprilTags::TagDetector* tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);
        vector<AprilTags::TagDetection> detections = tagDetector->extractTags(image_gray);

        // Sees how many tags were detected and references getDetectionString for each tag
        for (int i=0; i<detections.size(); i++) {
            AprilTags::TagDetection detection = detections[i];
            Eigen::Vector3d translation;
            Eigen::Matrix3d rotation;

            outputStrings[detection.id] = getDetectionString(detection, translation, rotation);
            lru[detection.id] = smoothingFrames; // Detected it again, so reset our cache
            vectorsFromCamera[detection.id] = translation;

            detection.draw(image); // Highlight in the video
        }

        // Maps how long a tag was last seen and if it can be discarded from print out. It
        // also measures the distance between tags using the identified reference tag. If
        // reference tag is removed, the distances default to last known values.
        map<int, int>::iterator it;
        for ( it = lru.begin(); it != lru.end(); it++){
            int key = it->first;  // string (key)
            int value = it->second;

            if(value < 0){ // Didn't get detected "in time", so it must be gone
                if(key == referenceTagId){
                    if(lastCount == 0){
                        cout << "Reference tag is out of sight, falling back on last known value." << endl;
                    }
                } else {
                    lru.erase(it);
                    outputStrings.erase(key);
                }
            } else { // Still detected. Continue as normal and print difference in vector distance in meters
                if(lastCount == 0){ 
                    Eigen::Vector3d newPosition = vectorsFromCamera[key] - vectorsFromCamera[referenceTagId];

                    cout << "Id: " << key
                    << ", distance=" << newPosition.norm()
                    << "m, x=" << newPosition(0)
                    << ", y=" << newPosition(1)
                    << ", z=" << newPosition(2) << endl;
                }
                lru[key] = value - 1;
            }
        }

        cv::imshow("Display Image", image); // Display the window

        if (cv::waitKey(10) == 27) break; // Exit if esc key is pressed
        
        cout << "\x1B[2J\x1B[H";
        lastCount = (lastCount + 1) % smoothingDisplay; // Counts to 10 milliseconds
    }

    return 0;
}