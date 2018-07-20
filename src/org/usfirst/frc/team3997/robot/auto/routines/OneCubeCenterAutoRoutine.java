package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.actions.CloseAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.IntakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.RelaxWristAction;
import org.usfirst.frc.team3997.robot.auto.actions.SwitchAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class OneCubeCenterAutoRoutine extends CommandGroup {

	public OneCubeCenterAutoRoutine() {
		addSequential(new CloseAction());
		addSequential(new WaitAction(Params.TIME_DELAY));
		addSequential(new RelaxWristAction());
		addSequential(new DriveDistanceAction(40, .6, 1, true));
		addSequential(new OuttakeAction(.2, 1));
		if(PlateDetector.getSwitchColor() == 'R') 
			addSequential(new DriveRotateAction(45, .6, 1.5, true));
		else
			addSequential(new DriveRotateAction(-45, .6, 1.5, true));
		addSequential(new DriveDistanceAction(64, .6, 3, true));
		addSequential(new IntakeAction(.8, 1));
		addSequential(new CloseAction());
		if(PlateDetector.getSwitchColor() == 'R') 
			addSequential(new DriveRotateAction(-45, .6, 1, true));
		else
			addSequential(new DriveRotateAction(45, .6, 1, true));
		addSequential(new SwitchAction());
		addSequential(new DriveDistanceAction(12, .5, .2, true));
		addSequential(new OuttakeAction(.8, -1));
		addSequential(new DriveDistanceAction(-20, .5, .8, true));


	}
	

}
