package org.usfirst.frc.team3997.robot.auto;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.actions.Action;
import org.usfirst.frc.team3997.robot.auto.actions.ArcadeStraightAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveIntervalAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;
import org.usfirst.frc.team3997.robot.auto.actions.IntakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.OuttakeAction;
import org.usfirst.frc.team3997.robot.auto.actions.VisionAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public abstract class AutoRoutine {
	public class OneCubeCenterAutoRoutine {

	}

	public boolean m_active = false;
	public boolean teleop;
	
	/*** Drives for a specified time interval
	 * 
	 * @param controllers Master Controllers for robot functionality
	 * @param seconds How long to drive
	 * @param y Forward Power
	 * @param x Rotational Power
	 */
	public void driveInterval(MasterController controllers, double seconds, double y, double x) {
		runAction(new DriveIntervalAction(controllers, seconds, y, x));
	}

	/*** Drives a specified Distance 
	 * 
	 * @param controllers Master Controllers for robot functionality
	 * @param desired_distance Desired setpoint (in inches)
	 * @param maxSpeed Max Speed of robot when driving
	 * @param timeout How much time allowed to take
	 * @param waitForTimeout Wait the timout even if pid has reached target?
	 */
	public void driveDistanceStraight(MasterController controllers, double desired_distance, double maxSpeed,
			double timeout, boolean waitForTimeout) {
		runAction(new DriveDistanceAction(controllers, desired_distance, maxSpeed, timeout, waitForTimeout));

	}
	/***
	 * Rotates robot with encoders
	 * @param controllers Master Controllers for robot functionality
	 * @param angle Angle of Rotation (in degrees)
	 * @param maxSpeed Max Speed of robot when driving
	 * @param timeout How much time allowed to take
	 * @param waitForTimeout Wait the timout even if pid has reached target?
	 */
	public void driveRotate(MasterController controllers, double angle, double maxSpeed, double timeout,
			boolean waitForTimeout) {
		runAction(new DriveRotateAction(controllers, angle, maxSpeed, timeout, waitForTimeout));
	}
	/*** Not currently implemented ***/
	public void vsionSetpointX(MasterController controllers, int setpoint, double maxSpeed,
			double timeout, boolean waitForTimeout) {
		runAction(new VisionAction(controllers, setpoint, maxSpeed, timeout, waitForTimeout));

	}
	/*** Uses arcade Drive to drive straighter
	 * 
	 * @param controllers Master Controllers for robot functionality
	 * @param distance desired_distance Desired setpoint (in inches)
	 * @param maxSpeed Max Speed of robot when driving
	 * @param timeout How much time allowed to take
	 * @param timeAfterHit Don't know
	 */
	public void arcadeDistanceStraight(MasterController controllers,
			double distance, double maxSpeed, double timeout, double timeAfterHit) {
		runAction(new ArcadeStraightAction(controllers, distance, maxSpeed, timeout, timeAfterHit));
	}
	
	/***
	 * Outtakes Power Up Cube
	 * @param controllers Master Controllers for robot functionality
	 * @param timeout How long to outtake?
	 * @param speed What speed?
	 */
	public void outtake(MasterController controllers, double timeout, double speed) {
		runAction(new OuttakeAction(controllers, timeout, speed));
	}
	/***
	 * Intakes Power Up Cube
	 * @param controllers Master Controllers for robot functionality
	 * @param timeout How long to intake?
	 * @param speed What speed?
	 */
	public void intake(MasterController controllers, double timeout, double speed) {
		runAction(new IntakeAction(controllers, timeout, speed));
	}
	

	/***
	 * Does nothing for given amount of time
	 * @param seconds How long to delay
	 */
	public void waitTime(double seconds) {
		runAction(new WaitAction(seconds));
	}
	
	/*** If autonomous structure is active ***/
	public boolean isActive() {
		return m_active;
	}
	/*** Runs auto routine ***/
	public void run() {
		m_active = true;
		routine();
	}
	/*** Stops auto Routine ***/
	public void stop() {
		m_active = false;
	}
	/*** Runs Action
	 * 
	 * @param action The action to run ***/
	public void runAction(Action action) {
		//Starts the action
		action.start();
		//TODO maybe add timer test this first
		double deltaTime = Timer.getFPGATimestamp();
		//If action is active and not finished, and the timer is less then 15 seconds, and it is in Autonomous and not disabled, then update action
		while((isActive()) && !(action.isFinished()) && (AutoRoutineRunner.getTimer().get() <= 15) && RobotState.isAutonomous() && !RobotState.isDisabled()) {
			action.update();
			deltaTime = Timer.getFPGATimestamp() - deltaTime;

		}
		SmartDashboard.putNumber("Control Loop Interation", deltaTime);
		//Finishes action
		action.finish();
	}
	
	/*** Abstract function that runs before we start autonomous ***/
	public abstract void prestart(); 
	/*** Abstract function that we run in autonomous ***/
	protected abstract void routine();
	
	
	
}
