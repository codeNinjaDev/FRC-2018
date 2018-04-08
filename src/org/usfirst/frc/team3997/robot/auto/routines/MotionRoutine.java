package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.MotionController;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class MotionRoutine extends AutoRoutine{
	MasterController controllers;
	Trajectory trajectory;
	Waypoint[] points = new Waypoint[] {
		    new Waypoint(0, 100, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
		};
	public MotionRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		trajectory = MotionController.generateTrajectory(points);
	}

	@Override
	protected void routine() {
		// TODO Auto-generated method stub
		
	}

}
