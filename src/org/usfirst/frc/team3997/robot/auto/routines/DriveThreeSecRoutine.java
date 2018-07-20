package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.auto.actions.DriveIntervalAction;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveThreeSecRoutine extends CommandGroup {
	public DriveThreeSecRoutine() {
		addSequential(new DriveIntervalAction(1, .25, 0));
	}


}
