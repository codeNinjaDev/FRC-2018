# FRC-2018
Base project for FRC-2018 

<h2>Structure</h2>
The code is organized into 5 main parts:
<li>
    <ul>Autonomous</ul>
    <ul>Controllers</ul>
    <ul>Feed</ul>
    <ul>Hardware</ul>
    <ul>PID</ul>
</li>
Robot, Params, and MasterController are located outside of these parts because they either use or are used by them.

<h3>Autonomous</h3>
Autonomous is split up into Actions and Routines.
<h4>Actions</h4>
Actions are commands that do a single task. For example, <b>Scale Action</b> moves the arm to scoring position on the scale.
<h4>Routines</h4>
Routines are command groups that work together to form an automous mode. A routine is a full autonomous that can be selected to run. 
<h5>Auto Selector</h5>
AutoSelector is a class that allows the routines to be selected from the SmartDashboard

<h3>Controllers</h3>
<h4>Controllers are classes that control a certain subsystem of the robot, such as the Hardware, arrm, and drivetrain.

<h3>Feed</h3>
<h4>Feed handles human parameter input, Game Data, and logging</h4>


<h3>Hardware</h3>
<h4>Handles all the sensors, motors, ports, and gamepads</h4>

<h3>PID</h3>
<h4>Classes to handle PID Source, Output, and Controllers</h4>

<h3>Robot</h3>
<h4>Main class of robot where everything is run.</h4>

<h3>Params</h3>
<h4>Parameters for robot, drivetrain, arm, etc..</h4>

<h3>MasterController</h3>
<h4>Getter and setter class for all controllers. Was absolutely necessary for recent, but may be deprecated due to switching to command-based</h4>


Pathfinder Library: https://github.com/JacisNonsense/Pathfinder/wiki/Pathfinder-for-FRC---Java
