/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.controllers.*;
import org.usfirst.frc.team3997.robot.feed.DataWriter;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import org.usfirst.frc.team3997.robot.MasterController;

/**
 * @author peter
 *
 */


public class DriveIntervalAction extends Action {
	private DriveController kDrive;
	private RobotModel robot;
	DataWriter<double[]> positionVsTimeCSV;
	public DriveIntervalAction(MasterController controllers, double seconds, double y, double x) {
		goal_time = seconds;
		x_drive = x;
		y_drive = y;
		this.kDrive = controllers.getDriveController();
		this.robot = controllers.getRobotModel();
		positionVsTimeCSV = new DataWriter<double[]>("PositionTime", double[].class);
	}
	
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() >= start_time + goal_time);
	}

	@Override
	public void update() {
		SmartDashboard.putNumber("reachedUPDATE", Timer.getFPGATimestamp());
		kDrive.arcadeDrive(y_drive, x_drive, false);
		double[] currentPos = {robot.leftDriveEncoder.getDistance(), robot.autoTimer.get()};
		positionVsTimeCSV.add(currentPos);
	}

	@Override
	public void finish() {
		kDrive.stop();
		double[] finalPos = {robot.leftDriveEncoder.getDistance(), robot.autoTimer.get()};
		positionVsTimeCSV.add(finalPos);
		robot.autoTimer.stop();
	}

	@Override
	public void start() {
		robot.resetEncoders();
		robot.autoTimer.start();
		double[] startPos = {robot.leftDriveEncoder.getDistance(), robot.autoTimer.get()};
		positionVsTimeCSV.add(startPos);
		SmartDashboard.putNumber("reachedSTART", Timer.getFPGATimestamp());
		start_time = Timer.getFPGATimestamp();
	}
}
