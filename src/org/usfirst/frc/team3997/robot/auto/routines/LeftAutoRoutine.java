/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import jaci.pathfinder.Trajectory;

/**
 * @author peter
 *
 */
public class LeftAutoRoutine extends AutoRoutine {
	private MasterController controllers;
	private ArmController arm;
	private RobotModel robot;

	public LeftAutoRoutine(MasterController controllers) {
		this.controllers = controllers;
		arm = controllers.getArmController();
	}

	@Override
	public void prestart() {
		controllers.getRobotModel().closeIntake();
	}

	@Override
	protected void routine() {
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
		arm.goToSwitchPosition();
		driveDistanceStraight(controllers, 103, .7, 3, true);
		driveRotate(controllers, 90, 1, 3, true);
		arm.outtakePowerCube();
		driveDistanceStraight(controllers, -30, .7, 5, true);
		arm.goToFeedPosition();
	}
	
	void passAutoLine() {
		driveDistanceStraight(controllers, 90, .7, 5, true);
	}
	
	void goToScale() {
		
		driveDistanceStraight(controllers, -191, .6, 7, true);
		robot.relaxWrist();
		waitTime(1);
		arm.goToScalePosition();
		arm.outtakePowerCube();
		waitTime(2);
		driveDistanceStraight(controllers, 30, .8, 4, true);
		arm.goToFeedPosition();
	}

}
