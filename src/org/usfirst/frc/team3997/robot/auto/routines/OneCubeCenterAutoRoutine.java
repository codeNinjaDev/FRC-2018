package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import edu.wpi.first.wpilibj.Timer;

public class OneCubeCenterAutoRoutine extends AutoRoutine {
	private MasterController controllers;

	public OneCubeCenterAutoRoutine(MasterController controllers) {
		this.controllers = controllers;
	}
	@Override
	public void prestart() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void routine() {
		controllers.getRobotModel().closeIntake();;
		controllers.getRobotModel().relaxWrist();;
		outtake(controllers, .2, 1);

		
		driveDistanceStraight(controllers, 47, .6, 3, true);
		//Timer.delay(.25);
		waitTime(0.5);
		if(PlateDetector.getSwitchColor() == 'R') 
			driveRotate(controllers, 45, .6, 2, true);
		else
			driveRotate(controllers, -45, .6, 2, true);
		waitTime(0.5);

		//Timer.delay(.25);
		driveDistanceStraight(controllers, 64, .6, 3, true);
		controllers.getArmController().goToSwitchPosition();
		if(PlateDetector.getSwitchColor() == 'R') 
			driveRotate(controllers, -45, .6, 2, true);
		else
			driveRotate(controllers, 45, .6, 2, true);
		driveDistanceStraight(controllers, 8, .5, 2, true);
		controllers.getRobotModel().outtakeBlock();
		
		controllers.getRobotModel().openIntake();
		waitTime(0.5);

		//Timer.delay(.25);
		controllers.getRobotModel().stopIntake();

		
		controllers.getDriveController().arcadeDrive(-.5, 0, false);
		waitTime(0.2);
		controllers.getDriveController().arcadeDrive(0, 0, false);

		

	}

}
