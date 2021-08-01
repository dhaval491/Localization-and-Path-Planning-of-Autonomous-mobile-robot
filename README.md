# Path Planning-of-Autonomous-mobile-robot
The MATLAB code that I have implemented for vision guided autonomous mobile robot. 
I have introduced a new Path PLanning Optimization Algorithm named: Next First Search Algorithm found in IJRSI Vol VI Issue XII (https://www.rsisinternational.org/journals/ijrsi/archive/volume-vi-issue-xii/#1516096414729-cab4f566-2fb8)

Algorithms used for comparisions: Ant Colony Optimization (ACO), Breadth First Search (BFS), Depth First Search (DFS) and Next First Search (NFS)

Link for the results and video of the project with NFS algorithm implemented: https://drive.google.com/open?id=16Xa8QiT5HuvIZ_-Q3bhyc2zQqa1HKA4G

**Tasks

**Capture the environment and detect robot and the obstacles. The detection is independent of indoor/outdoor environment and robust to the lighting effects.**

**The result is formed by fusing the data from features matching and image segmentation.**

**Estimate the current state of the robot**

**Input the destination on the live environment image**

**Calculate the shortest path avoiding the obstacles and send the signals in the form of distance and rotations to be performed in certain state**

Samlpe images

Raw image
![Algorithm implementation for detecting obstacles using segmentation](https://github.com/dhaval491/Path-Planning-of-Autonomous-mobile-robot/blob/master/IMG_20171231_090649.jpg)

Image after segmentation and making bounding boxes

![Segmented image and bounding boxes around the obstacles](https://github.com/dhaval491/Path-Planning-of-Autonomous-mobile-robot/blob/master/IMG_20171231_090631.jpg)

Sample calculated Shortest path path in real time. The stone is taken as destination. Video of working of the robot and its motion is in the link above

![Calculated shortest path](https://github.com/dhaval491/Path-Planning-of-Autonomous-mobile-robot/blob/master/path%20planing1.jpg)


