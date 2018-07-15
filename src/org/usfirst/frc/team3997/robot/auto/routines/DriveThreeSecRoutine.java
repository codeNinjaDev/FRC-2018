package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.actions.DriveIntervalAction;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveThreeSecRoutine extends CommandGroup {
	private MasterController controllers;
	
	public DriveThreeSecRoutine(MasterController controllers) {
		this.controllers = controllers;
		addSequential(new DriveIntervalAction(controllers, 3, 1, 0));


	}


}
