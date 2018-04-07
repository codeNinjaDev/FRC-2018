package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

public class RightSwitchRightSide extends AutoRoutine{
	RobotModel robot;
	ArmController arm;
	MasterController controllers;
	
	public RightSwitchRightSide(MasterController controllers) {
		arm = controllers.getArmController();
		robot = controllers.getRobotModel();
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void routine() {
	// Put cube in switch
			if (PlateDetector.getSwitchColor() == 'R') {
				goToSwitch();
			} else {
				//Drive Past Line
				driveDistanceStraight(controllers, 90, .7, 5, true);
			}
	}
	
	void goToSwitch() {
		arm.goToSwitchPosition();
		driveDistanceStraight(controllers, 103, .7, 5, true);
		arm.outtakePowerCube();
		driveDistanceStraight(controllers, -30, .7, 5, true);
		arm.goToFeedPosition();
	}
}
