/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.FeedAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.ScaleAction;
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
public class RighScale extends CommandGroup {
	private MasterController controllers;
	Trajectory trajectory;
	ArmController arm;
	RobotModel robot;
	public RighScale(MasterController controllers) {
		this.controllers = controllers;
		arm = controllers.getArmController();
		robot = controllers.getRobotModel();
		controllers.getRobotModel().closeIntake();
		addSequential(new WaitAction(Params.TIME_DELAY));
		boolean isLeftScale = (PlateDetector.getScaleColor() == 'R');
		if (isLeftScale) {
			goToScale();
			// trajectory = MotionController.generateTrajectory(rightRightPath);
		} else {
			passAutoLine();
		}
	}

	void passAutoLine() {
		addSequential(new DriveDistanceAction(controllers, 90, .7, 5, true));
	}

	void goToScale() {

		addSequential(new DriveDistanceAction(controllers, 298, .6, 4, true));
		addSequential(new WaitAction(1));
		addSequential(new DriveRotateAction(controllers, 90, .5, 2, false));
		robot.relaxWrist();
		addSequential(new WaitAction(1));
		robot.intakeWheels(-1);
		addSequential(new WaitAction(1));
		robot.stopIntake();
		addSequential(new ScaleAction(controllers));		
		addSequential(new OuttakeAction(controllers, 1, -1));
		addSequential(new FeedAction(controllers));


	}
}
