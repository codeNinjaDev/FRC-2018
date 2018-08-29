package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.actions.CloseAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.FeedAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.SwitchAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftSwitchLeftSide extends CommandGroup {
	
	public LeftSwitchLeftSide() {
		addSequential(new CloseAction());
		addSequential(new WaitAction(Params.TIME_DELAY));
		// Put cube in switch
		if (PlateDetector.getSwitchColor() == 'L') {
			goToSwitch();
		} else {
			//Drive Past Line
			passAutoLine();
		}
	}


	
	void goToSwitch() {
		addSequential(new SwitchAction());
		addSequential(new DriveDistanceAction(103, .7, 3, true));
		addSequential(new DriveRotateAction(-90, .6, 2, true));
		addSequential(new WaitAction(1.5));
		addSequential(new OuttakeAction(1, -1));
		addSequential(new DriveDistanceAction(-30, .7, 5, true));
		addSequential(new FeedAction());
	}
	
	void passAutoLine() {
		addSequential(new DriveDistanceAction(90, .7, 5, true));
	}

}
