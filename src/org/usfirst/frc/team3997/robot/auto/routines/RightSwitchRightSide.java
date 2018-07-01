package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.FeedAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.SwitchAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class RightSwitchRightSide extends CommandGroup {
	RobotModel robot;
	ArmController arm;
	MasterController controllers;

	public RightSwitchRightSide(MasterController controllers) {
		arm = controllers.getArmController();
		robot = controllers.getRobotModel();
		this.controllers = controllers;
		
		addSequential(new WaitAction(Params.TIME_DELAY));

		// Put cube in switch
		if (PlateDetector.getSwitchColor() == 'R') {
			goToSwitch();
		} else {
			// Drive Past Line
			addSequential(new DriveDistanceAction(controllers, 90, .7, 5, true));
		}
	}

	void goToSwitch() {
		addSequential(new SwitchAction(controllers));
		addSequential(new DriveDistanceAction(controllers, 103, .7, 3, true));
		addSequential(new DriveRotateAction(controllers, 90, .6, 2, true));
		addSequential(new WaitAction(1.5));
		addSequential(new OuttakeAction(controllers, 1, -1));
		addSequential(new DriveDistanceAction(controllers, -30, .7, 5, true));
		addSequential(new FeedAction(controllers));
	}
}
