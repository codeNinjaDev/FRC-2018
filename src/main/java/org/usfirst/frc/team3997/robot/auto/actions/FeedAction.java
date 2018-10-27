package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.controllers.ArmController;

import edu.wpi.first.wpilibj.command.Command;
/*** Moves the arm to "Rest" position */
public class FeedAction extends Command {
	/*** Arm */
	private ArmController arm;
	/*** Variable that checks whether arm has reached target position */
	private Boolean setpointReached;
	/*** Requires armController and sets variables
	 * 
	 */
	public FeedAction() {
		requires(Robot.armController);
		
		arm = Robot.armController;
		this.setpointReached = false;
	}
	@Override
	public boolean isFinished() {
		//Stops when Arm reaches setpoint. NO TIMEOUT, RISKY!!!
		//Who doesn't like risk?
		return setpointReached;
	}

	@Override
	public void execute() {
		
		if(arm.armPIDController.onTarget()) {
			setpointReached = true;
		} else {
			setpointReached = false;
		}
		
	}

	@Override
	public void end() {
		// TODO Auto-generated method stub
		arm.armPIDController.disable();
	}

	@Override
	public void initialize() {
		//Uses Arm Controller PID Function
		arm.goToFeedPosition();
	}

	protected void interrupted() {
		end();
	}
}
