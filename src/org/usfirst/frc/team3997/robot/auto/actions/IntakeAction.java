package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;

public class IntakeAction extends Action {
	double speed;
	RobotModel robot;
	public IntakeAction(MasterController controllers, double seconds, double speed) {
		goal_time = seconds;
		robot = controllers.getRobotModel();
		this.speed = speed;

	}
	
	public boolean isFinished() {
		return ((Timer.getFPGATimestamp() >= start_time + goal_time) || (!robot.getBlockTouching()));
	}

	@Override
	public void update() {
		robot.intakeWheels(speed);

	}

	@Override
	public void finish() {
		robot.closeIntake();
		robot.stopIntake();
	}

	@Override
	public void start() {
		start_time = Timer.getFPGATimestamp();
	}

}
