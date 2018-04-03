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
		boolean isLeftSwitch = (PlateDetector.getSwitchColor() == 'L');
		if(isLeftSwitch) {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			if(isLeftScale) {
				driveDistanceStraight(controllers, 100, 1, 5, true);
			} else {
				driveDistanceStraight(controllers, 240, .8, 6, true);
				driveRotate(controllers, -30, .5, 3, true);
				//arm.goToScalePosition();
				outtake(controllers, 4, 5);
			}
			
			// trajectory = MotionController.generateTrajectory(rightLeftPath);
		} else {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			if(isLeftScale) {
				driveDistanceStraight(controllers, 100, 1, 5, true);
			} else {
				driveDistanceStraight(controllers, 240, .8, 6, true);
				driveRotate(controllers, -30, .5, 3, true);
				//arm.goToScalePosition();
				outtake(controllers, 4, 5);
			}
			// trajectory = MotionController.generateTrajectory(rightRightPath);
		}
	}

	@Override
	protected void routine() {
		//pathFollower(controllers, trajectory, timeout);
	}


}
