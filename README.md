# AprilTag Distance Calculator: Using D435i RealSense
Calculate the distance between a reference AprilTag and additional AprilTags with a RealSense D435i depth camera. Output values referencing a defined AprilTag ID as the origin and track multiple AprilTags based on those coordinates.

Built with the a base of an open source [AprilTags C++ library](https://people.csail.mit.edu/kaess/apriltags/) and a reference to [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/librealsense), a hybrid distance calculator was made to reduce the startup costs of manually opening video streams, identify Apriltags while using the librealsense C++ API, and dynamically find the distance between tags with an [OpenCV](https://opencv.org/) display.

## Features

The main function of this program is to detect [AprilTags](https://github.com/AprilRobotics/apriltag) and print out the distances relative to an origin. Below is the main code segiment where the distance is printed relative to the vectors orginating from the camera in meters. This occurs only if the tags are detected in time. Oherwise, it brings up an error of lacking a reference tag where it defaults to the last known values or erases the entire string if a normal tag.

```c++
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
```

This structure can be seen best below as the tags are moved across the RealSense's field of view. The highlighted 



![](Demo.gif)


 * Code based off open source "Getting Started with OpenCV" with librealsense on GitHub and 
 * an open source AprilTags C++ library available on Linux and Mac. By using librealsense C++
 * as a high level API library, streaming from the RealSense is simplified by a few hundred lines
 * of code while adding additional features. These include modifying the reference point to an 
 * selected AprilTag and adding checks to this parameter. The output is carriage returned and
 * cleared in the terminal as it continuously updates in a pop up window using OpenCV. Smoothing 
 * was added for the continuous updates as every so often a tag wouldn't be recognized in a frame. 
 * By using the librealsense API, additional use of point cloud referencing and checks would be simple 
 * to implement and layer, but have yet to be included.



## Installation

### Requirements



## Usage

### Command-Line Interface & OpenCV

### Use Case



## Future Improvements




You can use one `#` all the way up to `######` six for different heading sizes.

If you'd like to quote someone, use the > character before the line:

> Coffee. The finest organic suspension ever devised... I beat the Borg with it.
> - Captain Janeway

I think you should use an
`<Add in here>` element here instead.

    function fancyAlert(arg) {
      if(arg) {
        $.facebox({div:'#foo'})
      }
    }

![](realsensesize.jpg)

![](Demo.gif)
