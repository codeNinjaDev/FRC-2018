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
public class LeftScale extends CommandGroup {
	private MasterController controllers;
	Trajectory trajectory;
	ArmController arm;
	RobotModel robot;

	public LeftScale(MasterController controllers) {
		this.controllers = controllers;
		arm = controllers.getArmController();
		robot = controllers.getRobotModel();
		controllers.getRobotModel().closeIntake();
		addSequential(new WaitAction(Params.TIME_DELAY));
		boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
		if (isLeftScale) {
			addSequential(new LeftScale(controllers));
			// trajectory = MotionController.generateTrajectory(rightRightPath);
		} else {
			passAutoLine();
		}
	}

	void passAutoLine() {
		addSequential(new DriveDistanceAction(controllers, 90, .7, 5, true));
	}


}
