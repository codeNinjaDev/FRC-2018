/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import jaci.pathfinder.Trajectory;

/**
 * @author peter
 *
 */
public class LeftScale extends AutoRoutine {
	private MasterController controllers;
	Trajectory trajectory;
	ArmController arm;
	RobotModel robot;
	public LeftScale(MasterController controllers) {
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
		waitTime(Params.TIME_DELAY);
		boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
		if (isLeftScale) {
			goToScale();
			// trajectory = MotionController.generateTrajectory(rightRightPath);
		} else {
			passAutoLine();
		}
	}
	

	
	
	void passAutoLine() {
		driveDistanceStraight(controllers, 90, .7, 5, true);
	}
	
void goToScale() {
		
		driveDistanceStraight(controllers, 298, .6, 4, true);
		waitTime(1);
		driveRotate(controllers, -90, .5, 2, false);
		robot.relaxWrist();
		waitTime(1);
		robot.intakeWheels(-1);
		waitTime(1);
		robot.stopIntake();
		goToScale(controllers);
		//waitTime(2.5);
		outtake(controllers, 1, -1);
		goToFeed(controllers);

		
	}


}
