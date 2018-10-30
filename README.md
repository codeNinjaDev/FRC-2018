# FRC-2018
Base project for FRC-2018 

<h1>Structure</h1>
The code is organized into 5 main parts:
<ul>
    <li>Autonomous</li>
    <li>Controllers</li>
    <li>Feed</li>
    <li>Hardware</li>
    <li>PID</li>
</ul>
Robot, Params, and MasterController are located outside of these parts because they either use or are used by them.

<h2>Autonomous</h3>
Autonomous is split up into Actions and Routines.
<h3>Actions</h4>
Actions are commands that do a single task. For example, <b>Scale Action</b> moves the arm to scoring position on the scale.
<h3>Routines</h4>
Routines are command groups that work together to form an automous mode. A routine is a full autonomous that can be selected to run. 
<h4>Auto Selector</h5>
AutoSelector is a class that allows the routines to be selected from the SmartDashboard

<h2>Controllers</h3>
Controllers are classes that control a certain subsystem of the robot, such as the Hardware, arrm, and drivetrain.

<h2>Feed</h3>
Feed handles human parameter input, Game Data, and logging


<h2>Hardware</h3>
Handles all the sensors, motors, ports, and gamepads

<h2>PID</h3>
Classes to handle PID Source, Output, and Controllers

<h2>Robot</h3>
Main class of robot where everything is run.

<h2>Params</h3>
Parameters for robot, drivetrain, arm, etc..

<h2>MasterController</h3>
Getter and setter class for all controllers. Was absolutely necessary for recent, but may be deprecated due to switching to command-based

<h1>Outside Libraries/files</h3>:
Pathfinder Library: https://github.com/JacisNonsense/Pathfinder/wiki/Pathfinder-for-FRC---Java

Also used CSV Writer from team254

