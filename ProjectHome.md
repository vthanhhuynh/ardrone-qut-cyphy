# Project Description #
This project present monocular vision guided autonomous navigation system for Micro Aerial Vehicles (MAVs) in GPS-denied environments.
The main problem of a monocular system is recovering a scale of a scene which can not be determined without prior knowledge or other sensors. To address this problem we solve a cost function, which consists of absolute drift free altitude measurement together with up-to-scaled position estimation from visual sensor.

We also evaluate the proposed system in terms of : accuracy of scale estimator, controller performance and accuracy of state estimation by comparing with motion capture ground truth. All resources including source code, tutorial documentation and system models are available online.


# AR.Drone coordinate system #
<img src='http://wiki.ardrone-qut-cyphy.googlecode.com/git/imgs/ardrones.png' width='500'>

<h1>ROS Software Block Diagram</h1>
<img src='http://wiki.ardrone-qut-cyphy.googlecode.com/git/imgs/software_blockdiagram.png' width='500'>

<h1>PTAM Results</h1>
<img src='http://wiki.ardrone-qut-cyphy.googlecode.com/git/imgs/ptam_test.png' />

<h1>Video Demonstration</h1>
<a href='http://www.youtube.com/watch?feature=player_embedded&v=zvNhFcbvXSI' target='_blank'><img src='http://img.youtube.com/vi/zvNhFcbvXSI/0.jpg' width='425' height=344 /></a>