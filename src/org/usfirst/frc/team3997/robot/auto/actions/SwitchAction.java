package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.controllers.ArmController;

import edu.wpi.first.wpilibj.command.Command;

public class SwitchAction extends Command {
	private ArmController arm;
	private Boolean setpointReached;
	public SwitchAction() {
		requires(Robot.armController);
		arm = Robot.armController;
		
		this.setpointReached = false;
	}
	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
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
		
	}

	@Override
	public void initialize() {
		arm.goToSwitchPosition();
	}
	protected void interrupted() {
		end();
	}

}
