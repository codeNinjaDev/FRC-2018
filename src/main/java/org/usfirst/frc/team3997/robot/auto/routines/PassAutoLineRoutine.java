package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PassAutoLineRoutine extends CommandGroup {
	
	public PassAutoLineRoutine() {
		addSequential(new DriveDistanceAction(90, .7, 5, true));
	}
}
