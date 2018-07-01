package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

public class LeftSwitchLeftSide extends AutoRoutine {
	RobotModel robot;
	ArmController arm;
	MasterController controllers;

	public LeftSwitchLeftSide(MasterController controllers) {
		arm = controllers.getArmController();
		robot = controllers.getRobotModel();
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void routine() {
		waitTime(Params.TIME_DELAY);
		// Put cube in switch
		if (PlateDetector.getSwitchColor() == 'L') {
			goToSwitch();
		} else {
			//Drive Past Line
			passAutoLine();
		}

	}
	void goToSwitch() {
		arm.goToSwitchPosition();
		driveDistanceStraight(controllers, 103, .7, 3, true);
		driveRotate(controllers, -90, .6, 2, true);
		waitTime(1.5);
		outtake(controllers, 1, -1);
		driveDistanceStraight(controllers, -30, .7, 5, true);
		arm.goToFeedPosition();
	}
	
	void passAutoLine() {
		driveDistanceStraight(controllers, 90, .7, 5, true);
	}

}
