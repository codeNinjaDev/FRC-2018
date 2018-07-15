package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoNothingRoutine extends CommandGroup {

	public DoNothingRoutine() {
		SmartDashboard.putString("Nothing", "TRUE");
		addSequential(new WaitAction(15));
	}
	

}
