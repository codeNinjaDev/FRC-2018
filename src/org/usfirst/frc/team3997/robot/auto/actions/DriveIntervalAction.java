/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.controllers.*;
import org.usfirst.frc.team3997.robot.feed.DataWriter;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Robot;

/**
 * @author peter
 *
 */


public class DriveIntervalAction extends Command {
	private DriveController kDrive;
	private RobotModel robot;
	
	double goal_time, x_drive, y_drive, start_time;
	DataWriter<double[]> positionVsTimeCSV;
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
	
	public void interrupt() {
		end();
	}
	
	protected void interrupted() {
		end();
	}
}
