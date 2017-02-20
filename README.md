#Kinect ROS Client
This program receives messages from a Microsoft Kinect Sensor on a Windows computer and sends the messages to a Linux computer which converts them to a ROS format.The data from the messages is then analyzed and can be used to determine behaviors of the bodies sensed by the kinect.

Most code taken from [CMU Original Code - Client](https://github.com/personalrobotics/k2_client) 

###Constructing the Workspace

This repository should be added to a source folder inside of a catkin workspace

Example path: folder/workspace/src/k2_client

The Dockerfile should be placed in 'folder'


`catkin_make` your workspace before starting the program

##Running Docker
ROS Master, all ROS nodes, and the rosbridge server are all intended to be run within a [Docker](https://www.docker.com/) container. 

###Dockerfile
Make sure the dockerfile includes:

`FROM ros:indigo-ros-base`

`RUN apt-get install sudo`

`RUN sudo apt-get update`

`RUN sudo apt-get -y install build-essential`

`RUN sudo apt-get -y install wget`

`RUN sudo apt-get -y install ros-indigo-ros-tutorials`

`RUN sudo apt-get -y install ros-indigo-tf`

`RUN sudo apt-get -y install libsndfile1-dev libpng12-dev`

`RUN sudo apt-get -y install libyaml-cpp-dev`

`RUN sudo apt-get -y install ros-indigo-cv-bridge`

`RUN sudo apt-get -y install ros-indigo-image-transport`

`RUN sudo apt-get -y install ros-indigo-camera-info-manager`

`RUN sudo apt-get -y install ros-indigo-octomap-msgs`

`RUN sudo apt-get -y install ros-indigo-nao-robot`

`RUN sudo apt-get -y install ros-indigo-nao-extras`

`RUN sudo apt-get -y install ros-indigo-rosbridge-server`

`WORKDIR /root/workspace`

1. Open a Docker terminal
2. Navigate (`cd`) to one directory above your workspace directory 
3. Execute `docker build -t [directory_above_workspace] .`  Now, you've built the docker container that we'll soon run. 
4. Get your current absolute path (`pwd`)
5. Run your docker container with the following command (remember to put in your absolute path):
   
   `docker run -it -v [absolute_path]/[your workspace]:/root/[your workspace] [directory_above_workspace]`

6. Run the following commands:

`catkin_make`

`source ~/[your workspace]/devel/setup.bash`


##How to start up another bash terminal from a pre-existing Docker container
1. Open another Docker terminal
2. Execute `docker ps`
3. Find the name of the container from which you wish to run another bash terminal 
4. Execute `docker exec -it [container_name] bash`
5. Run the following commands:

`source /opt/ros/indigo/setup.bash`

`source ~/workspace/devel/setup.bash`

`source ./devel/setup.bash`


##Running the program
1. Check the ip address of the windows computer by running `ipconfig` in a new terminal on the windows computer. 
2. Navigate to the launch file on the linux computer, located in `launch`
3. Change the value parameter of each node to be the ip address of the windows computer, and save the file
4. In a new terminal window, run `roslaunch k2_client k2_client.launch`

If the connection is successful, the Windows computer output should indicated that it has added the client and is successfully sending messages

**Make sure that the [server program](https://github.com/ScazLab/k2_server) is running on the windows side**

##Translating incoming JSON messages to ROS messages
There are six programs that each listen to a port for incoming kinect data on the linux computer. Bodies.cpp, Audio.cpp, and Faces.cpp are used in this program. They listen on ports 9003, 9004, and 9005 for body, audio, and face data, and once data is received, the data is converted to a ROS Node and published on individual topics. The message format is determined by body, audio, and face messages. Port specifications can be changed on the windows computer in `Settings.Designer.cs` and the ports can be changed on the linux side from bodies.cpp, audio.cpp, and faces.cpp.

Every time changes are made to any of the src files or the msg files, `catkin_make` the workspace

#Data Parsing

###Running Data Parser:
Data parser subscribes to the audio, body, and face topics, parses through the data and calculates certain behaviors, then publishes a message on a new topic. To run data-parser.py, navigate to scripts, `chmod +x data-parser.py`. Now the program can be run from the workspace. `cd ..` then `rosrun k2_client data-parser.py`

###Publisher Component of data-parser
After all information is received and analyzed, the data needed is published to a new topic ('Information'). The message is sent as a PersonArray, which is a PersonMessage array. PersonMessage contains specific fields:

+ x, y, and z position based on the head
+ seat
+ looking at
+ head pitch, roll and yaw orientation
+ volume and is talking
+ leaning, face touch, hand touch, and arms crossed
+ time the message was sent

In order to receive the person information, `rostopic echo Information`

##Seat Assignment
'Seats' are assigned at the beginning of the program to each body based on the position within the first 10 messages sent. Each person object has a seat. **Warning: Every person must be in a seat before the program starts**

##Keeping Track of Changing Tracking Ids
If bodies move out of the frame of the kinect or they switch places, they are given a new tracking id. In order to preserve the data, the program matches the new 'untracked' body with one of the bodies previously tracked. It first tries to match a person based on their last known position. If this does not succeed, the program attempts to match the current position of the body with the known position of a seat. If the program is not able to match a tracking id to a person, it continues to try and prints out `Error: Untracked Body`

#####Functions below are calculated in the program based on data returned by the kinect, not by the kinect itself. Error values were chosen based on trial and error. They are values that allow for detection of certain actions without returning false positives.

##Body
###Arms Crossed
Returns true if the absolute value of the distance between the x, y, and z coordinates of:

+ left elbow and right hand (joints 5 and 23)
+ right elbow and left hand (joints 9 and 21)

are both less than an error range of .3 meters

###Hand Touch
Returns true if the absolute value of the distance between the x, y, and z coordinates of the right hand and left hand are less than an error range of .15 meters

The kinect is not usually able to differentiate Hand Touch and Arms Crossed and commonly loses track of one or both hands

###Face Touch
Returns true if the absolute value of the distance between the x, y, and z coordinates of the left hand **OR** right hand and the head are less than an error range of .35 meters

###Lean
Returns 'Forward' if the z orientation of the spine (joint 1) is less than -.03 and 'Backward' if the orientation of the spine is greater than .01

##Face
###Looking At
Person A looking at Person B is determined by calculating the ideal yaw of A. There are 12 cases that need to be considered based on which side of the kinect people are positioned and if they are farther or closer to the kinect

+ If both are on the negative side and the x distance between B and A is negative, add 180 degrees (pi radians) to the calculated ideal yaw angle
+ If both are on the positive side and the x distance between B and A is positive, subtract 180 degrees from the previously calculated ideal yaw angle
+ In all other cases, there is no change necessary for ideal yaw:
 * If both are on the negative side and the x distance between B and A is positive
 * If both are on the postivie side and the x distance between B and A is negative
 * If one is on the postive side and one is on the negative side

If the absolute value of the difference of the ideal yaw angle and the yaw of the person's head is less than an error range of .6 radians and the absolute value of the difference between the ideal pitch angle and the pitch of the person's head is less than an error range of .4 radians, `A.looking_at_seat` is changed to the seat of B


Note: The kinect usually cannot detect faces if they turn past a y orientation of about 200 degrees (pos or neg) or x orientation of 70 degrees (pos or neg)


##Audio
###Volume
The algorithm for determining volume uses a RMS of the audioStream. Code for this was taken from the `Reader_SpeechFrameArrived` method in `KinectClient.cs` from [this program](https://github.com/ScazLab/rocket_collaboration/tree/master/Thalamus/ThalamusKinectV2.0) written by Andre Pereira.
###Total Talking Time
Audio messages are received every 16 milliseconds if the kinect detects audio within that time frame. For this reason, every time the kinect detects audio and the linux machine receives and audio message, total talking time (recorded in milliseconds) is incremented by 16.


#Bugs/TODO
+ When running for an extended period (usually within 10 minutes), all programs terminate with a 'connection reset by peer error'
+ Kinect accuracy is poor when people are sitting, especially for hand positions