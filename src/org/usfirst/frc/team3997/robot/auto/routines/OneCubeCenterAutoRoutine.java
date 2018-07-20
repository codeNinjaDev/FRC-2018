package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.actions.CloseAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveIntervalAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.OpenAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.RelaxWristAction;
import org.usfirst.frc.team3997.robot.auto.actions.StopAction;
import org.usfirst.frc.team3997.robot.auto.actions.SwitchAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OneCubeCenterAutoRoutine extends CommandGroup {
	private MasterController controllers;

	public OneCubeCenterAutoRoutine(MasterController controllers) {
		this.controllers = controllers;
		addSequential(new WaitAction(Params.TIME_DELAY));
		addSequential(new CloseAction(controllers));
		addSequential(new RelaxWristAction(controllers));
		

		// previous 47 in
		addSequential(new DriveDistanceAction(controllers, 40, .6, 1, true));
		addSequential(new OuttakeAction(controllers, 0.2, 1));
		// Timer.delay(.25);
		addSequential(new WaitAction(0.5));
		if (PlateDetector.getSwitchColor() == 'R')
			addSequential(new DriveRotateAction(controllers, 45, .6, 1.5, true));
		else
			addSequential(new DriveRotateAction(controllers, -45, .6, 1.5, true));

		// Timer.delay(.25);
		addSequential(new DriveDistanceAction(controllers, 64, .6, 3, true));
		addSequential(new OuttakeAction(controllers, .8, 1));
		addSequential(new SwitchAction(controllers));
		addSequential(new CloseAction(controllers));
		if (PlateDetector.getSwitchColor() == 'R')
			addSequential(new DriveRotateAction(controllers, -45, .6, 1.5, true));
		else
			addSequential(new DriveRotateAction(controllers, 45, .6, 1.5, true));
		
		addSequential(new SwitchAction(controllers));
		addSequential(new CloseAction(controllers));
		addSequential(new DriveDistanceAction(controllers, 12, .5, 2, true));;
		// It is reversed
		addSequential(new WaitAction(0.5));
		addSequential(new OpenAction(controllers));
		addSequential(new OuttakeAction(controllers, 1, -1));
		addSequential(new WaitAction(0.25));	
		addSequential(new StopAction(controllers));
		addSequential(new DriveIntervalAction(controllers, .8, -.6, 0));

	}

}
