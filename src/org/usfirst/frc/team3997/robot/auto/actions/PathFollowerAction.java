package org.usfirst.frc.team3997.robot.auto.actions;

import java.util.Date;
import java.util.concurrent.Callable;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.MotionController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
/*** Autonomous Action that follows trajectory ***/
public class PathFollowerAction extends Action{
	private MotionController motion;
	private RobotModel robot;
	private double timeout;
	/****
	 * Constructor for PathFollower Action
	 * @param controllers All the controllers for robot functionality
	 * @param trajectory Pathfinder @jaci Trajectory to follow
	 * @param timeout Time allowed to follow path
	 */
	public PathFollowerAction(MasterController controllers, Trajectory trajectory, double timeout) {
		this.motion = controllers.getMotionController();
		this.robot = controllers.getRobotModel();
		this.timeout = timeout;
		SmartDashboard.putString("MOTIONPROFILING", "SETTING_UP");
		//Sets up configuration, modifiers, and encoder followers
		this.motion.setUp(trajectory);
		SmartDashboard.putString("MOTIONPROFILING", "FINISHED_SETTING_UP");

	}
	@Override
	/*** Checks if Trajectory is finished or we went over timeout ***/
	public boolean isFinished() {
		return (motion.isProfileFinished()) || (Timer.getFPGATimestamp() >= start_time + timeout);
	}
	

	@Override
	/*** Runs in loop and updates PIDVA motion pid controller ***/
	public void update() {
		
	}

	@Override
	/*** Disables motion profiling when finished ***/
	public void finish() {
		motion.disable();
	}

	@Override
	/*** Starts Profile following ***/
	public void start() {
		//Starts timer
		start_time = Timer.getFPGATimestamp();
		//Sets up sensors
		robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		robot.resetGyro();
		robot.leftDriveEncoder.reset();
		robot.rightDriveEncoder.reset();
		//Starts following path
		motion.enable();
		ScheduledExecutorService scheduledExecutorService = Executors.newScheduledThreadPool(5);

		Runnable motionRunnable = new Runnable() {
			
			public void run() {
				motion.update();
				if (motion.isProfileFinished() || (Timer.getFPGATimestamp() >= start_time + timeout)) {
					scheduledExecutorService.shutdown();
				}
			}
		};
		
		scheduledExecutorService.scheduleAtFixedRate(motionRunnable, 0, 20, TimeUnit.MILLISECONDS);
	}

}
