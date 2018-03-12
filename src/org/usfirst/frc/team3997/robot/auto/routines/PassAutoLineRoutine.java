package org.usfirst.frc.team3997.robot.auto.routines;

import java.io.File;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.MotionController;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class PassAutoLineRoutine extends AutoRoutine {
	private MasterController controllers;
	double deltaTime =0;
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
		deltaTime = Timer.getFPGATimestamp();
		traj = MotionController.generateTrajectory(point);
		deltaTime = Timer.getFPGATimestamp() - deltaTime;
		//System.out.println(traj);
		
	}

	@Override
	protected void routine() {
		SmartDashboard.putNumber("Delta Pathfinder Time", deltaTime);

		pathFollower(controllers, traj, 10);
		SmartDashboard.putString("MOTIONPROFILING", "FINISHED_RUNNING");


	}
}
