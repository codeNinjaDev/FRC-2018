package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;

public class OuttakeAction extends Action {

	double speed;
	RobotModel robot;
	public OuttakeAction(MasterController controllers, double timeout, double speed) {
		goal_time = timeout;
		robot = controllers.getRobotModel();
		this.speed = speed;
		
	}
	
	public boolean isFinished() {
		return ((Timer.getFPGATimestamp() >= start_time + goal_time) || (robot.getBlockTouching()));
	}

	@Override
	public void update() {
		robot.intakeWheels(-speed);

	}

	@Override
	public void finish() {

		//robot.outtakeBlock();
		robot.openIntake();
		robot.stopIntake();

	}

	@Override
	public void start() {
		robot.flexWrist();
		start_time = Timer.getFPGATimestamp();
	}

}
