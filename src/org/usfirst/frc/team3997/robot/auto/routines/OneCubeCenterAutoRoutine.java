package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
		waitTime(Params.TIME_DELAY);
		controllers.getRobotModel().closeIntake();;
		controllers.getRobotModel().relaxWrist();;

		//previous 47 in 
		driveDistanceStraight(controllers, 40, .6, 1, true);
		outtake(controllers, .2, 1);
		//Timer.delay(.25);
		waitTime(0.5);
		if(PlateDetector.getSwitchColor() == 'R') 
			driveRotate(controllers, 45, .6, 1.5, true);
		else
			driveRotate(controllers, -45, .6, 1.5, true);

		//Timer.delay(.25);
		driveDistanceStraight(controllers, 64, .6, 3, true);
		outtake(controllers, .8, 1);
		controllers.getArmController().goToSwitchPosition();
		controllers.getRobotModel().closeIntake();
		if(PlateDetector.getSwitchColor() == 'R') 
			driveRotate(controllers, -45, .6, 1, true);
		else
			driveRotate(controllers, 45, .6, 1, true);
		controllers.getArmController().goToSwitchPosition();
		controllers.getRobotModel().closeIntake();
		driveDistanceStraight(controllers, 12, .5, 2, true);
		//It is reversed
		waitTime(0.5);
		controllers.getRobotModel().openIntake();
		outtake(controllers, 1, -1);
		waitTime(0.25);
		//Timer.delay(.25);
		controllers.getRobotModel().stopIntake();

		
		controllers.getDriveController().arcadeDrive(-.5, 0, false);
		waitTime(.8);
		controllers.getDriveController().arcadeDrive(0, 0, false);

		

	}

}
