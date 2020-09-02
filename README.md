Sample-DepthCamera
=============

*SolAR is an open-source framework released under Apache license 2.0 making possible to easily create your own camera pose estimation solution to develop Augmented Reality applications. 
SolAR is dedicated to Augmented Reality (AR).
It offers a C++ SDK to easily and quickly develop and use custom solutions for camera pose estimation. It provides developers with a full chain from low-level vision components development to camera pose estimation pipelines and AR service development.*

The **SolAR Depth sample** show a SolAR pipeline for augmented reality based using [ Intel® RealSense™](https://www.intelrealsense.com/) depth camera


## How run it

### StandAlone
* To run it, first print the 3D [Bunny](./StandAlone/bunny_10000_opencv.ply) mesh.
* Put it in center front of the camera (at least > 20cm)
* Open a terminal and execute `./bin/Release/SolARDepthCameraSampleStandAlone.exe`
* Press `R` to 
* Press `escape` to quit the application
* If you want to change properties of the components of the pipeline, edit the [conf_DepthCamera.xml](./StandAlone/conf_DepthCamera.xml) file


| ![](./depth_rgb.jpg) | ![](./depth_lut.jpg) | ![](./depth_3dview.jpg) | 
|:-:|:-:|:-:| 
| RGB | 2D Depth Map | 3D Depth Map |

### Plugin 
*Unity plugin coming soon*

## Contact 
Website https://solarframework.github.io/

Contact framework.solar@b-com.com


