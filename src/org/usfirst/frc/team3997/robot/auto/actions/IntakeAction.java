package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class IntakeAction extends Command {
	double speed, goal_time, start_time;
	RobotModel robot;
	public IntakeAction(MasterController controllers, double seconds, double speed) {
		requires(controllers.getRobotModel());

		goal_time = seconds;
		robot = controllers.getRobotModel();
		this.speed = speed;

	}
	
	protected boolean isFinished() {
		return ((Timer.getFPGATimestamp() >= start_time + goal_time)/* || (!robot.getBlockTouching())*/);
	}

	protected void execute() {
		robot.intakeWheels(-speed);

	}

	protected void end() {
		robot.closeIntake();
		robot.stopIntake();
	}

	protected void intialize() {
		robot.relaxWrist();
		start_time = Timer.getFPGATimestamp();
	}

	protected void interrupt() {
		end();
	}
}
