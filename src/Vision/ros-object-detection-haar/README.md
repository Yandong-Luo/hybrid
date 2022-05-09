# ros-object-detection-haar

Object detection package for ROS that uses Haar cascade classifiers. Requires OpenCV. Useful for simple face detection, cat detection, etc. on crappy embedded hardware. You can get 15 fps+ on a Raspberry Pi.

## Try it

```rosrun object_detection_haar object_detection_haar_node __ns:=/camera```

## Parameters:

* **cascade** (string) -- name of the cascade to use. Must be an OpenCV-compatible Haar cascade in the nodes/haar directory.
* **topic_image** (string) -- topic to listen for images. Defaults to "image_raw".
* **topic_detections** (string) -- topic to output semantic predictions. Defaults to "detections". Outputs a JSON string which is a list of [[x0,y0,w0,h0], [x1,y1,w1,h1], ...].

## Subscribers:

* **image_raw** (sensor_msgs/Image)

## Publishers:

* **detections** (std_msgs/String)

## Disclaimer

This is provided "as-is" for educational purposes. I am not liable for any damage or injury that may result from the use of this software.
