package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;

public class OuttakeAction extends Action {

	double speed;
	RobotModel robot;
	public OuttakeAction(MasterController controllers, double seconds, double speed) {
		goal_time = seconds;
		robot = controllers.getRobotModel();
		this.speed = speed;
		
	}
	
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() >= start_time + goal_time);
	}

	@Override
	public void update() {
		robot.outtakeBlock(speed);

	}

	@Override
	public void finish() {
		robot.outtakeBlock(0);
	}

	@Override
	public void start() {
		start_time = Timer.getFPGATimestamp();
	}

}