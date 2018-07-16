package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.ArmController;

import edu.wpi.first.wpilibj.command.Command;

public class ScaleAction extends Command {
	private ArmController arm;
	private Boolean setpointReached;
	public ScaleAction(MasterController controllers) {
		arm = controllers.getArmController();
		requires(this.arm);

		this.setpointReached = false;
	}

	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return setpointReached;
	}

	protected void execute() {
		if(arm.armPIDController.onTarget()) {
			setpointReached = true;
		} else {
			setpointReached = false;
		}
		
	}

	protected void end() {
		arm.armPIDController.disable();
		
	}

	protected void intialize() {
		arm.goToScalePosition();
	}
	
	protected void interrupt() {
		end();
	}

}
