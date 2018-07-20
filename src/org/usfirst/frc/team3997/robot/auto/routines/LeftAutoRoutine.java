/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.auto.actions.CloseAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.FeedAction;
import org.usfirst.frc.team3997.robot.auto.actions.IntakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.RelaxWristAction;
import org.usfirst.frc.team3997.robot.auto.actions.ScaleAction;
import org.usfirst.frc.team3997.robot.auto.actions.SwitchAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author peter
 *
 */
public class LeftAutoRoutine extends CommandGroup {


	public LeftAutoRoutine() {
		addSequential(new CloseAction());
		
		boolean isLeftSwitch = (PlateDetector.getSwitchColor() == 'L');
		if (isLeftSwitch) {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			boolean isRightScale = (PlateDetector.getScaleColor() == 'R');
			/***
			 * LEFT Side:
			 * 
			 * If Left Switch: If Right Scale: Go TO SWITCH: Arm to switch pos Drive 140
			 * inches turn 90 degrees drive 20 inches outtake block If Left Scale: Drive
			 * -240 inches to the scale Rotate 30 degrees Arm to Scale Pos Outtake Block
			 * 
			 ***/
			if (isLeftScale) {
				goToScale();
			} else if (isRightScale) {
				goToSwitch();
			}

			// trajectory = MotionController.generateTrajectory(rightLeftPath);
		}
		boolean isRightSwitch = (PlateDetector.getSwitchColor() == 'R');
		if (isRightSwitch) {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			boolean isRightScale = (PlateDetector.getScaleColor() == 'R');

			if (isLeftScale) {
				goToScale();
			} else if (isRightScale) {
				/***
				 * Right Side:
				 * 
				 * If Right Switch: 
				 * 
				 * If Right Scale: PASS AutoLine Drive Forward 200 in Outtake
				 * Block 
				 * 
				 * If Left Scale: Go TO SCALE: Arm to switch pos Drive 240 inches turn 30
				 * degrees outtake block
				 * 
				 ***/
				passAutoLine();
			}
			// trajectory = MotionController.generateTrajectory(rightRightPath);
		}
	}

	
	
	void goToSwitch() {
		addSequential(new SwitchAction());
		addSequential(new DriveDistanceAction(103, .7, 3, true));
		addSequential(new RelaxWristAction());
		addSequential(new DriveRotateAction(-90, 1, 3, true));
		addSequential(new IntakeAction(1, -1));
		addSequential(new DriveDistanceAction(-30, .7, 5, true));
		addSequential(new FeedAction());
	}
	
	void passAutoLine() {
		addSequential(new DriveDistanceAction(90, .7, 5, true));
	}
	
	void goToScale() {
		addSequential(new DriveDistanceAction(298, .6, 4, true));
		addSequential(new WaitAction(1));
		addSequential(new DriveRotateAction(90, .5, 2, false));
		addSequential(new RelaxWristAction());
		addSequential(new WaitAction(1));
		addSequential(new IntakeAction(1, -1));
		addSequential(new ScaleAction());
		addSequential(new OuttakeAction(1, 1));
		addSequential(new FeedAction());
	}

}
