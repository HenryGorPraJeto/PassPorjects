This file contains all the MATLAB scripts and ros package for the final RRT Path Planning project

*The pfms_support folder contain the stage simulator which already set up the enviroment

*The Obstacle check folder is a ROS package that perform the obstacle-checking function of the project

*The Bio_lastest.m file is the Bi-directional RRT path Planning MATLAB script what this script do is the perform Bi-directional path planning and publish planned path information on ROS

*The projectlatest_19_10.m file is the Normal RRT path planning MATLAB script what this script do is the perform normal RRT path planning and publish Planned path information on ROS

*The controller2.m File is the Controller of the robot what this script do is to subcribes to the Topic that publish by RRT or Bi-directional RRT program to obtains the planned path information then control the robot follow the planned path to reach the final goal pose

instruction:
1. open stage-ros simulartor (roslaunch a4_setup a4_setup.launch ) and move robot 0 around to construct OG-map
2. rosrun the obstacle-detector package (rosrun)
3. run RRT or Bi-directional RRT script (run script in MATLAB)
4. Once path planning done run the controller script to move robot.(Run script in MATLAB)
