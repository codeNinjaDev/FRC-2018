package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

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
		driveDistanceStraight(controllers, 66, .6, 3, true);
		if(PlateDetector.getSwitchColor() == 'R') 
			driveRotate(controllers, 60, .6, 3.5, true);
		else
			driveRotate(controllers, -60, .6, 3.5, true);
		driveDistanceStraight(controllers, 88.35, .6, 3, true);
		outtake(controllers, 2, .5);
	}

}
