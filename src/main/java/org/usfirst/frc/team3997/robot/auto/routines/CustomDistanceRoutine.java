 package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.actions.DriveIntervalAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.feed.DashboardVariables;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CustomDistanceRoutine extends CommandGroup {
	public CustomDistanceRoutine() {
		addSequential(new WaitAction(Params.TIME_DELAY));
		addSequential(new DriveIntervalAction(DashboardVariables.firstAutoTime, 1, 0));
	}
	

}