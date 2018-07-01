/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.FeedAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.ScaleAction;
import org.usfirst.frc.team3997.robot.auto.actions.SwitchAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;

/**
 * @author peter
 *
 */
public class RightAutoRoutine extends CommandGroup {
	private MasterController controllers;
	Trajectory trajectory;
	ArmController arm;
	RobotModel robot;
	public RightAutoRoutine(MasterController controllers) {
		this.controllers = controllers;
		arm = controllers.getArmController();
		robot = controllers.getRobotModel();
		
		boolean isLeftSwitch = (PlateDetector.getSwitchColor() == 'L');
		if (isLeftSwitch) {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			boolean isRightScale = (PlateDetector.getScaleColor() == 'R');
			/***
			 * Right Side:
			 * 
			 * If Left Switch: If Right Scale: Drive -240 inches to the scale Rotate 30
			 * degrees Arm to Scale Pos Outtake Block If Left Scale: Cross Auto Line: Arm to
			 * switch pos Drive 200 inches
			 * 
			 ***/
			if (isLeftScale) {
				passAutoLine();
			} else if (isRightScale) {
				addSequential(new RighScale(controllers));
			}

			// trajectory = MotionController.generateTrajectory(rightLeftPath);
		}
		boolean isRightSwitch = (PlateDetector.getSwitchColor() == 'R');
		if (isRightSwitch) {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			boolean isRightScale = (PlateDetector.getScaleColor() == 'R');

			if (isLeftScale) {
				goToSwitch();
				;
			} else if (isRightScale) {
				addSequential(new RighScale(controllers));
			}
			// trajectory = MotionController.generateTrajectory(rightRightPath);
		}
	}

	
	

	void goToSwitch() {
		addSequential(new SwitchAction(controllers));
		addSequential(new DriveDistanceAction(controllers, 103, .7, 3, true));
		robot.relaxWrist();
		addSequential(new DriveRotateAction(controllers, 90, 1, 3, true));
		addSequential(new OuttakeAction(controllers, 1, 1));
		addSequential(new DriveDistanceAction(controllers, -30, .7, 5, true));
		arm.goToFeedPosition();
	}

	void passAutoLine() {
		addSequential(new DriveDistanceAction(controllers, -90, .7, 5, true));
	}

	


}
