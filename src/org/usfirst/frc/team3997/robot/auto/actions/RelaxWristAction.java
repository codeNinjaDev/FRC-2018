package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.command.Command;

public class RelaxWristAction extends Command {
	RobotModel robot;

	public RelaxWristAction() {
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
		robot.relaxWrist();
	}
	
	protected void interrupted() {
		end();
	}
}

