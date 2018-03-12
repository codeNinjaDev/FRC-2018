package org.usfirst.frc.team3997.robot.auto.routines;

import java.io.File;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.MotionController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class PassAutoLineRoutine extends AutoRoutine {
	private MasterController controllers;
	Waypoint[] point = new Waypoint[] {
			new Waypoint(0,0,0),
			new Waypoint(100,0,0)
	};
	Trajectory traj;
	public PassAutoLineRoutine(MasterController controllers) {
		this.controllers = controllers;
		
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		File myFile = new File("/home/lvuser/mynewtrajectory.csv");
		traj = Pathfinder.readFromCSV(myFile);
		//System.out.println(traj);
		
	}

	@Override
	protected void routine() {
		SmartDashboard.putString("MOTIONPROFILING", "RUNNING");

		pathFollower(controllers, traj, 10);
		SmartDashboard.putString("MOTIONPROFILING", "FINISHED_RUNNING");


	}
}
