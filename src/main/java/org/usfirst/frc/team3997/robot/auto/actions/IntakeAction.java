package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class IntakeAction extends Command {
	double speed, start_time, goal_time;
	RobotModel robot;
	public IntakeAction(double seconds, double speed) {
		requires(Robot.robot);
		
		goal_time = seconds;
		robot = Robot.robot;
		this.speed = speed;

	}
	
	public boolean isFinished() {
		return ((Timer.getFPGATimestamp() >= start_time + goal_time)/* || (!robot.getBlockTouching())*/);
	}

	@Override
	public void execute() {
		robot.intakeWheels(-speed);

	}

	@Override
	public void end() {
		robot.closeIntake();
		robot.stopIntake();
	}

	public void interrupt() {
		end();
	}
	
	@Override
	public void initialize() {
		robot.relaxWrist();
		start_time = Timer.getFPGATimestamp();
	}

	protected void interrupted() {
		end();
	}
}
