As mentioned in the lecture, you need to calibrate your camera and generate a calibration file for the intrinsics. You also need to estimate the transform from the center of the robot to the camera R^T_C
.

A good reference for OpenCV-based calibration is available here: https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html

A good ROS2 camera calibration package is available here: https://docs.ros.org/en/kilted/p/camera_calibration/doc/tutorial_mono.html

Camera Issue Fix
Updated 11 hours ago by Charvi Shukla and Shiladitya Biswas
Endorsed by Instructor (Shiladitya Biswas)
Hi everyone, we were able to debug the camera no detected issue for the robots. The fix is quiet easy and straight forward. Go this this git repo (https://github.com/notu97/rubikpi_flash) and just copy all the files starting with com. to the “/usr/lib/qcm6490/camera/” directory on your Rubik Pi 3 .

git clone https://github.com/notu97/rubikpi_flash.git
cd rubikpi_flash
sudo cp com.qti.sensor.ov5647.so* com.qti.sensormodule.cam*_ov5647.bin /usr/lib/qcm6490/camera/
Visit Manage Class to disable runnable code snippets×
Then reboot the rubik pi 3 and try to run the template robot_vision code launch file. Open foxglove page on your laptop and subscribe to the camera topic. You should see some images. If you see a jagged image, its probably some calibration issue. But the no camera detected issue should be fixed.

Alternatively, you can also download the folder from this Google Drive link: https://drive.google.com/drive/folders/123Oc7OlRl4OBIdABYAqQ6tPJmkyXhoI0?usp=sharing

Step 0: You have likely already done this, but make sure you can do ssh access to your robot.
Step 1: Download the folder from that Google Drive link
Step 2: Copy it over to your RUBIK Pi using the following command:
rsync -avz --rsync-path="sudo rsync" <PATH TO camera-driver-files on local> ubuntu@<IP ADDRESS>:/usr/lib/qcm6490/camera/
Step 3: Put in the password for ubuntu when prompted.
Step 4: ssh into your robot and check if the files are present in /usr/lib/qcm6490/camera/
Step 5: Log out of the robot. Power it off, count till 10 and then restart.

Now we have the camera driver files and we should be able to use the starter code. To ensure we have all dependencies for it, do the following:

Run: sudo apt update
Run sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libopencv-dev
Make sure ROS2 is sourced: source /opt/ros/jazzy/setup.bash
Go inside your ROS2 workspace: cd ~/ros2\_ws
Build the camera package provided to you in the starter code: colcon build --packages-select robot_vision_camera
Source this code: source install/setup.bash
In one tmux shell launch the camera: ros2 launch robot_vision_camera robot_vision_camera.launch.py
In another tmux shell: ros2 topic list
You should now be able to see the camera topics.

You can also go on https://app.foxglove.dev/, sign in, and then click “Open Connection”. Update the link to: “ws://<Your Robot’s IP>:8765”. Add an image panel and set the topic to /camera/image_raw and calibration to None. You should be able to see your camera feed now.

Let us know if this works out for you and feel free to reach out if there are any questions!