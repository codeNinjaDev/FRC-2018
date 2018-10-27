package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.hardware.RobotHardware;

import edu.wpi.first.wpilibj.command.Command;

public class OpenAction extends Command{
	RobotHardware robot;

	OpenAction() {
		requires(Robot.robot);
		this.robot = Robot.robot;
	}
	
	/*** Checks if action is finished ***/
	public boolean isFinished() {
		//Check if we want to only wait for Timeout
		return true;
	}

	@Override
	/*** Updates action code in a loop ***/
	public void execute() {
		
	}

	@Override
	/*** Code that runs when action ends ***/
	public void end() {
		
	}

	@Override
	/*** Starts action ***/
	protected void initialize() {
		robot.closeIntake();
	}
	
	protected void interrupted() {
		end();
	}
}
