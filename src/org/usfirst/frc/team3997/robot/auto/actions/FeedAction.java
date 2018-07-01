package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.ArmController;

public class FeedAction extends Action {
	private ArmController arm;
	private Boolean setpointReached;
	public FeedAction(MasterController controllers) {
		arm = controllers.getArmController();
		this.setpointReached = false;
	}
	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return setpointReached;
	}

	@Override
	public void update() {
		if(arm.armPIDController.onTarget()) {
			setpointReached = true;
		} else {
			setpointReached = false;
		}
		
	}

	@Override
	public void finish() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void start() {
		arm.goToScalePosition();
	}

}
