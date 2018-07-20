/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.actions.CloseAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.FeedAction;
import org.usfirst.frc.team3997.robot.auto.actions.IntakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.RelaxWristAction;
import org.usfirst.frc.team3997.robot.auto.actions.ScaleAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author peter
 *
 */
public class RightScale extends CommandGroup {
	public RightScale() {
	addSequential(new CloseAction());
	addSequential(new WaitAction(Params.TIME_DELAY));
	boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
	if (isLeftScale) {
		goToScale();
		// trajectory = MotionController.generateTrajectory(rightRightPath);
	} else {
		passAutoLine();
	}
}


void passAutoLine() {
	addSequential(new DriveDistanceAction(90, .7, 5, true));
}

void goToScale() {

	addSequential(new DriveDistanceAction(298, .6, 4, true));
	addSequential(new WaitAction(1));
	addSequential(new DriveRotateAction(-90, .5, 2, false));
	addSequential(new RelaxWristAction());
	addSequential(new WaitAction(1));
	addSequential(new IntakeAction(1, -1));
	addSequential(new ScaleAction());
	addSequential(new OuttakeAction(1, 1));
	addSequential(new FeedAction());

}


}
