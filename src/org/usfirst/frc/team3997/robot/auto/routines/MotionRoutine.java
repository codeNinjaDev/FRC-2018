package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.MotionController;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class MotionRoutine extends CommandGroup{
	MasterController controllers;
	Trajectory trajectory;
	Waypoint[] points = new Waypoint[] {
		    new Waypoint(0, 100, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
		};
	public MotionRoutine() {
		trajectory = MotionController.generateTrajectory(points);
	}

	

}
