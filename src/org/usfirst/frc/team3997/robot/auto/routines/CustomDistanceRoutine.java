 package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.feed.DashboardVariables;

public class CustomDistanceRoutine extends AutoRoutine {
	MasterController controllers;
	public CustomDistanceRoutine(MasterController controllers) {
		this.controllers = controllers;
	}
	@Override
	public void prestart() {
		
	}

	@Override
	protected void routine() {
		waitTime(Params.TIME_DELAY);
		driveInterval(controllers, DashboardVariables.firstAutoTime, 1, 0);
	}

}