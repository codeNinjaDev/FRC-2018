/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import jaci.pathfinder.Trajectory;

/**
 * @author peter
 *
 */
public class RightAutoRoutine extends AutoRoutine{
	private MasterController controllers;
	Trajectory trajectory;
	ArmController arm;
	public RightAutoRoutine(MasterController controllers) {
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
		if(isLeftSwitch) {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			boolean isRightScale = (PlateDetector.getScaleColor() == 'R');
			/*** 
			 * Right Side:
			 * 
			 * 	If Left Switch:
			 *	 If Right Scale:
			 * 		Drive -240 inches to the scale
			 * 		Rotate 30 degrees
			 * 		Arm to Scale Pos
			 * 		Outtake Block
			 * 	 If Left Scale:
			 * 		Cross Auto Line:
			 * 		Arm to switch pos
			 * 		Drive 200 inches
			 * 
			 * ***/
			if(isLeftScale) {
				driveDistanceStraight(controllers, 200, 1, 5, true);
			} else if(isRightScale){
				driveDistanceStraight(controllers, -240, .8, 6, true);
				driveRotate(controllers, 30, .5, 3, true);
				arm.goToScalePosition();

				outtake(controllers, 4, .75);
				controllers.getRobotModel().openIntake();;
			}
			
			// trajectory = MotionController.generateTrajectory(rightLeftPath);
		} 
		boolean isRightSwitch = (PlateDetector.getSwitchColor() == 'R');
		if(isRightSwitch){
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			boolean isRightScale = (PlateDetector.getScaleColor() == 'R');
			
			if(isLeftScale) {
				arm.goToSwitchPosition();
				driveDistanceStraight(controllers, 140, 1, 5, true);
				driveRotate(controllers, -40, .6, 3, true);
				outtake(controllers, 3, .75);
				controllers.getRobotModel().openIntake();;
			} else if(isRightScale){
				/*** 
				 * Right Side:
				 * 
				 * 	If Right Switch:
				 *	 If Right Scale:
				 * 		Drive -240 inches to the scale
				 * 		Rotate 30 degrees
				 * 		Arm to Scale Pos
				 * 		Outtake Block
				 * 	 If Left Scale:
				 * 		Go TO SWITCH:
				 * 		Arm to switch pos
				 * 		Drive 140 inches
				 * 		turn -90 degrees
				 * 		drive 20 inches
				 * 		outtake block
				 * 
				 * ***/
				driveDistanceStraight(controllers, -240, .8, 6, true);
				driveRotate(controllers, 30, .5, 3, true);
				arm.goToScalePosition();

				outtake(controllers, 4, .75);
				controllers.getRobotModel().openIntake();;
			}
			// trajectory = MotionController.generateTrajectory(rightRightPath);
		}	}


}
