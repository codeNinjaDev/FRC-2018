package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.command.Command;

public class OpenAction extends Command {
	RobotModel robot;
	
	public OpenAction(MasterController controllers) {
		requires(controllers.getRobotModel());

		this.robot = controllers.getRobotModel();
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
		robot.openIntake();
	}
	
	protected void interrupted() {
		end();
	}



	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}
}
