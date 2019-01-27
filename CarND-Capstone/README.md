# Programming a Real Self-Driving Car

## Writeup

**Implement a Self-Driving Algorithm**

The goals / steps of this project are the following:
- Implement a self-driving controller algorithm following the ROS architecture
- Integrate a pre-trained traffic light classifier to identify the traffic signals

[//]: # (References)
[image1]: ./imgs/final-project-ros-graph-v2.png "ROS"
[image2]: ./imgs/sample_simulator.jpg "Simulator"
[image3]: ./imgs/labeling.jpg "LabelImg"
[image4]: ./imgs/sample_site.jpg "Site"
[image5]: ./imgs/simulator_result1.png "Simulator Result 1"
[image6]: ./imgs/simulator_result2.png "Simulator Result 2"
[image7]: ./imgs/site_result1.png "Site Result 1"
[image8]: ./imgs/site_result2.png "Site Result 2"
[image9]: ./imgs/simulator.png "Simulator Result"

## Team Member
This project is completed individually by Fengwen Song (<onlyenos@gmail.com>).

## Rubric Points
 Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1140/view) individually and describe how I addressed each point in my implementation.

## Software Architecture

The testing platform was pre-installed with ROS, which is the fundamental architecture for the self-driving algorithms. Details of the architecture can be found in the following image.

![ROS Architecture][image1]

Here arrows with lables represent the subscription or topics that passes among functional blocks. And each functional block represents a module that is inplemented in Python. The functional blocks to implement during this project are:

| Name | Description | 
|:---:|:---:| 
| [Traffic light detector](./ros/src/tl_detector/tl_detector.py) | Implementation of call-backs for camera images and detection of traffic lights |
| [Traffic light classifier](./ros/src/tl_detector/light_classification/tl_classifier.py) | Implementation of classifier that loads pre-trained model to detect traffic lights from images received |
| [Waypoint updater](./ros/src/waypoint_updater/waypoint_updater.py) | Implementation of waypoints generator that create waypoints for the vehicle to follow depending on whether traffic light was observed and map data |
| [DBW node](./ros/src/twist_controller/dbw_node.py) | Implementation of drive-by-wire module that generate control signals (throttle, brake, steering) based on the given waypoints |
| [Twist controller](./ros/src/twist_controller/twist_controller.py) | Implementation of the detailed control functions that are called by dbw node to generat the control signals based on the idea of yaw, PID controller |

Apart from the ROS nodes, 2 frozen tensor files that are pre-trained with the traffic light images from simulator and real test site are included in the project repository and will be called by the algorithm for traffic light identification

| Name | Description | 
|:---:|:---:| 
| [Simulator](./ros/src/tl_detector/light_classification/simulator_fine_tune_2000/frozen_inference_graph.pb) | Model that is trained based on simulator screenshots |
| [Site](./ros/src/tl_detector/light_classification/fast_rcnn_incept_3000/frozen_inference_graph.pb) | Model that is trained basd on ROS bag image data |

Details of the classifier will be covered in the sections below.

## Waypoint Updater

Implementation of the waypoint update is based on the [walk-through videos](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/e4ed7b44-6330-48a2-bfb0-fd65fff1b4d1) provided by Udacity. The waypoint updater node subscribes to the topic ``/base_waypoints`` and it adopts the KDTree library to sort the waypoints based on their distance to the current vehicle location. In the meantime, it will generate the lane containing the following ``LOOKAHEAD_WPS`` numbers of waypoints in front of the vehicle at the rate of 50 Hz.

When generating the lane, it will also subscribe to the ``\traffic_waypoint`` topic to determine whether red traffic light is detected in front of the vehicle. It will follow the waypoint speed limit if traffic is clean ([line 92](./ros/src/waypoint_updater/waypoint_updater.py)) or it will generate a constant-deaccleration speed profile to make sure vehicle is able to fully stop in front of the red traffic light ([line 94](./ros/src/waypoint_updater/waypoint_updater.py)).

## Drive-by-Wire Node

Implementation of the ``dbw_node`` is based on the [walk-through videos](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/6546d82d-6028-4210-a4b0-9d559662a881) provided by Udacity. WIthin its initialization functions it create a controller object defined by ``twist_controller`` node and initialize key vehicle parameters including vehicle mass, dimensions and dynamic limits.

During the drive it will send out control signals including throttle, brake and steering calculated by ``twist_controller`` object at the rate of 50 Hz, which is requested by the actuator on the car and simulator. In the meantime it will check whether the drive-by-wire bit is enable and will stop sending control signals and reset the PID controller whenever this bit is disabled (usually done by the test staff on the vehicle under emergency)

The ``twist_controller`` node contains objects of a PID controller, yaw controller and low-pass filter. Both PID and yaw controllers were implemented by Udacity together with the low-pass filter. This node will first filter the vehicle speed signal it received with the low-pass filter, and send it to both PID and yaw controllers to calculate the necessary control signals so that the vehicle is able to follow the desired speed and track.

## Classifier

### Platform

The traffic light classifier was trained with [Google Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection). It is an opn-source framework built on top of Tensorflow and supports multiple common networks that are pre-trained for object detection.

Here since the version of tensorflow on testing vehicle is 1.3.0, the commit [1e2ada](https://github.com/hamediramin/ObjectDetectionAPI/commit/1e2ada24c6734b3f6f4e09cb98f66f3aad68de76) is adopted instead of the latest commit.

### Training Dataset

It is recommended that different models are trained and selected separately by the algorithm for simulator and site test to achieve the best performance. Thus different methods were adopted in this project to acquire training dataset for either scenario.

#### Simulator

The [simulator](https://github.com/udacity/CarND-Capstone/releases) is programmed based on Unity 3D engine and can interact with the self-driving algorithm via COM port 4567 communication. From the architecture above it can be found that the topic ``/image_color`` is passed by the simulator to the traffic light detector node and contains the camera images captured. Therefore the simplest way to acquire training dataset is to modify the traffic light call-back function and save the images received.

In this project 302 images were captured and stored locally as the training dataset containing red, yellow and green lights at different distance (see a sample of captured image in simulator below)
![Simulator camera image sample][image2]

To label the location and type of traffic lights in the image, the tool [labelImg](https://github.com/tzutalin/labelImg) is adopted in this project. This tool support quick labeling and catagorizing the objects in an image and can generate label files in the format supported by Tensorflow Object Detection API. Below is a screenshot of the tool when labeling a sample image.
![LabelImg][image3]

#### Site

For real site testing data, Udacity provides a sample [rosbag file](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view) that is recorded in the same testing site. This rosbag file contains a crop of video recorded with camera on Carla together with the vehicle location information and can be serve as the training dataset for site scenario.

To retrieve image from the rosbag file, the ROS node ``image_view`` is used here with the following command:

```sh
rosrun image_view image_saver _sec_per_frame:=0.01 image:=/image_raw
```

It saves the video frame when the rosbag file is played with the frame rate specified. In this project 480 image files were used as the training dataset containing red, yellow and green lights at different distance (see a sample of captured image in simulator below)
![Carla camera image sample][image4]

All images were then labeled by LabelImg tool in the same way as simulator case.

#### Dataset Conversion

After labeling, the training dataset now contains camera files in image format and labeling files in .xml format in the following hierarchy:

```
path/to
|
└─rgb
│   │  img01.jpg
│   │  img02.jpg
│   │  ...
|   |
│   └──labels
│      │   img01.xml
│      │   img02.xml
│      │   ...
|
```

They need to be converted into a specific type of file that can be adopted by Tensorflow Object Detection API as the training dataset. Object Detection API includes the relevant functions of conversion in the repository, and for this project the [to_tfrecords.py](https://github.com/bosch-ros-pkg/bstld/blob/master/tf_object_detection/to_tfrecords.py) function included in the [Bosch Small Traffic Lights Dataset](https://github.com/bosch-ros-pkg/bstld) was modified to perform the conversion.

### Selection of Neural Network and Training

Tensorflow Object Detection API has a variety supports of networks included in the [Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md). It is recommended from peer reviews ([Alex](https://github.com/alex-lechner/Traffic-Light-Classification#21-extract-images-from-a-rosbag-file) and [Intel](https://software.intel.com/en-us/articles/traffic-light-detection-using-the-tensorflow-object-detection-api)) that the following 2 models might suit the best in this project:

* [SSD Inception V2 Coco (01/28/2018)](http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2018_01_28.tar.gz)
* [Faster RCNN Inception V2 Coco (01/28/2018)](http://download.tensorflow.org/models/object_detection/mask_rcnn_inception_v2_coco_2018_01_28.tar.gz)

Both models are pre-trained networks with [coco dataset](http://cocodataset.org/#home) and can be fine-trained in this project based on the idea of transfer learning. After comparison, [Faster RCNN Inception V2 Coco (01/28/2018)](http://download.tensorflow.org/models/object_detection/mask_rcnn_inception_v2_coco_2018_01_28.tar.gz), though has to be trained separately for simulator and site scenarios, achieved better accuracy (over 90%) than [SSD Inception V2 Coco (01/28/2018)](http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2018_01_28.tar.gz) (around 60%) and is thus selected as the model of training in the project.

Fine-trainings were performed on [AWS EC2 instances](https://us-west-1.console.aws.amazon.com/ec2/v2/home?region=us-west-1#). The AMI created by Udacity ``udacity-carnd-advanced-deep-learning`` was selected as the template and ``g2.8xlarge`` was selected as the instanc type to ensure better performance. 2000 steps of training was performed for simulator training dataset and 3000 steps for the site.

After training the tensor file was frozen into .pb files and downloaded from AWS instance to local drive so that the self-driving algorithm can access. Verification was performed on the local gaming laptop with the following hardware configuration:

| Component | Details |
|:---:|:---:| 
| CPU | Intel I7 6820HK |
| RAM | 32GB DDR4 2133 |
| GPU | Nvidia GTX 980 |

When testing with the simulator, no delay was observed for traffic light detection, which indicates the time cost of detection model is less than the sampling rate of the camera. It can be then assumed that no performance issue will be expected from the real site testing as the [hardware configuration](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/project) on the testing vehicle is more powerful than what mentioned above.

### Results of Detection

Models trained under both simulator and site training dataset achieved great accuracy (see images below):

<img src=./imgs/simulator_result1.png height=330> <img src=./imgs/simulator_result2.png height=330>
*Simulator results*

<img src=./imgs/site_result1.png height=330> <img src=./imgs/site_result2.png height=330>
*Site results*

<img src=./imgs/simulator.png>
*Simulator Screenshot*

It can be observed from the sample above that traffic lights are identified clearly with high confidency regardless of distance and the outline of the traffic light was marked accurately as well. Simulator test also proves that vehicle is able to react correctly according to the varying traffic light conditions.

## Notes from the Reviewer

#### Please attach any comments or questions below this line:




