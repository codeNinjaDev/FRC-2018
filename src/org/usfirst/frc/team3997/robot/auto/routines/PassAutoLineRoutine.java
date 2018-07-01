package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PassAutoLineRoutine extends CommandGroup {
	private MasterController controllers;
	
	public PassAutoLineRoutine(MasterController controllers) {
		this.controllers = controllers;
		addSequential(new DriveDistanceAction(controllers, 90, .7, 5, true));

	}

}
