package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.ArmController;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwitchAction extends Command {
	private ArmController arm;
	private Boolean setpointReached;
	public SwitchAction(MasterController controllers) {
		arm = controllers.getArmController();
		this.setpointReached = false;
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return setpointReached;
	}

	@Override
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
		arm.goToSwitchPosition();
		SmartDashboard.putString("PID Switch", "RUnning");
	}
	
	protected void interrupt() {
		end();
	}

}
