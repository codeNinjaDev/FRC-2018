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
public class RightAutoRoutine extends AutoRoutine {
	private MasterController controllers;
	Trajectory trajectory;
	ArmController arm;
	RobotModel robot;
	public RightAutoRoutine(MasterController controllers) {
		this.controllers = controllers;
		arm = controllers.getArmController();
		robot = controllers.getRobotModel();
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
				goToScale();
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
				goToScale();
			}
			// trajectory = MotionController.generateTrajectory(rightRightPath);
		}
	}
	

	void goToSwitch() {
		arm.goToSwitchPosition();
		driveDistanceStraight(controllers, -103, .7, 3, true);
		robot.relaxWrist();
		driveRotate(controllers, 90, 1, 3, true);
		arm.outtakePowerCube();
		driveDistanceStraight(controllers, -30, .7, 5, true);
		arm.goToFeedPosition();
	}
	
	void passAutoLine() {
		driveDistanceStraight(controllers, -90, .7, 5, true);
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
