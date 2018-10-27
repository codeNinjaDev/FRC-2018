/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.feed.DataWriter;
import org.usfirst.frc.team3997.robot.hardware.RobotHardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author peter
 *
 */

/*** Drive for a time interval. Includes code to log data to a file on the Roborio */
public class DriveIntervalAction extends Command {
	/*** Drivetrain */
	private DriveController kDrive;
	/*** Hardware */
	private RobotHardware robot;
	/*** Global variables that control the movement of the robot. */
	double goal_time, x_drive, y_drive, start_time;
	/*** Variable that logs to CSV file on robot */
	DataWriter<double[]> positionVsTimeCSV;
	/***
	 * Constructor for Driving for a time interval
	 * @param seconds Number of seconds to drive
	 * @param y Power [-1, 1] Forward 
	 * @param x Power [-1, 1] Turning 
	 */
	public DriveIntervalAction(double seconds, double y, double x) {
		
		requires(Robot.driveController);
		requires(Robot.robot);
		
		goal_time = seconds;
		x_drive = x;
		y_drive = y;
		this.kDrive = Robot.driveController;
		this.robot = Robot.robot;
		
		positionVsTimeCSV = new DataWriter<double[]>("/home/lvuser/PositionTime.csv", double[].class);
		System.out.println("Action Drive ");
	}
	/*** Has time finished? */
	@Override
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() >= start_time + goal_time);
	}


	@Override
	public void execute() {
		SmartDashboard.putNumber("reachedUPDATE", Timer.getFPGATimestamp());
		kDrive.arcadeDrive(y_drive, x_drive, false);
		double[] currentPos = {robot.leftDriveEncoder.getDistance(), robot.autoTimer.get()};
		positionVsTimeCSV.add(currentPos);
		System.out.println("UPDATING");
		SmartDashboard.putString("AUTON", "UPDATING");
	}

	@Override
	public void end() {
		kDrive.stop();
		double[] finalPos = {robot.leftDriveEncoder.getDistance(), robot.autoTimer.get()};
		positionVsTimeCSV.add(finalPos);
		positionVsTimeCSV.flush();
		robot.autoTimer.stop();
	}

	@Override
	public void initialize() {
		robot.resetEncoders();
		robot.autoTimer.start();
		double[] startPos = {robot.leftDriveEncoder.getDistance(), robot.autoTimer.get()};
		positionVsTimeCSV.add(startPos);
		SmartDashboard.putNumber("reachedSTART", Timer.getFPGATimestamp());
		start_time = Timer.getFPGATimestamp();
	}
	
	protected void interrupted() {
		end();
	}
}
