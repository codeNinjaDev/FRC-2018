/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.actions.CloseAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.FeedAction;
import org.usfirst.frc.team3997.robot.auto.actions.IntakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.RelaxWristAction;
import org.usfirst.frc.team3997.robot.auto.actions.ScaleAction;
import org.usfirst.frc.team3997.robot.auto.actions.StopAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import com.sun.xml.internal.ws.util.pipe.StandalonePipeAssembler;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;

/**
 * @author peter
 *
 */
public class RightScale extends CommandGroup {
	private MasterController controllers;
	
	public RightScale(MasterController controllers) {
		this.controllers = controllers;
		
		addSequential(new CloseAction(controllers));
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
		addSequential(new RelaxWristAction(controllers));
		addSequential(new WaitAction(1));
		addSequential(new OuttakeAction(controllers, 1, 1));
		addSequential(new StopAction(controllers));
		addSequential(new ScaleAction(controllers));		
		addSequential(new OuttakeAction(controllers, 1, -1));
		addSequential(new FeedAction(controllers));


	}
}
