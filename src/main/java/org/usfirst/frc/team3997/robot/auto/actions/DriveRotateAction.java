package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.hardware.RobotHardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Rotate to a specified angle with encoders as sensor ***/
public class DriveRotateAction extends Command{
	private DriveController driveTrain;
	private RobotHardware robot;
	/*** Distance Setpoint (Converted from angle in degrees to inches) ***/
	private double distance;
	/*** Max time for action to complete ***/
	private double timeout, start_time;
	/*** Max speed action can run ***/
	private double maxSpeed;
	//P I D constants
	/** PID coefficients 
	 */
	private double P, I, D;
	/*** The residual distance of encoders (error basically) ***/
	private double leftEncoderStartDistance, rightEncoderStartDistance;
	/*** Checks if reached desired setpoint ***/
	private boolean reachedSetpoint;
	/*** If true, it waits until timeout is complete, even if PID has hit the setpoint ***/
	private boolean waitForTimeout;
	
	/***
	 * Converts angle to degrees and initalizes all variables
	 * @param controllers ALl classes that control robot functionality
	 * @param angle Angle Setpoint (e.g 90 degrees)
	 * @param maxSpeed Max Speed of rotation (-1 to 1)
	 * @param timeout Max allowed time action runs for
	 * @param waitForTimeout Whether to wait the full timeout, even if the setpoint is reached
	 */ 
	public DriveRotateAction(double angle, double maxSpeed, double timeout, boolean waitForTimeout) {
		
		requires(Robot.driveController);
		requires(Robot.robot);
		
		this.driveTrain = Robot.driveController;
		//It takes 20 inches on the left side and -20 inches on the right side to turn 90 degrees
		this.distance = (angle * 20.0) / (90.0);
		this.timeout = timeout;
		this.robot = Robot.robot;
		this.maxSpeed = maxSpeed;
		this.waitForTimeout = waitForTimeout;
		
		reachedSetpoint = false;
		leftEncoderStartDistance = 0.0; 
		rightEncoderStartDistance = 0.0;
		
		P = 1;
		// TODO Might Need to change pID values
		I = Params.drive_i;
		D =  .1;
		
		SmartDashboard.putNumber("DRIVE_PID_P", P);
		SmartDashboard.putNumber("DRIVE_PID_I", I);
		SmartDashboard.putNumber("DRIVE_PID_D", D);

	}
	@Override
	public boolean isFinished() {
		if((Timer.getFPGATimestamp() >= start_time + timeout) && !(reachedSetpoint)) {
			
		}
		if(waitForTimeout) 
			return (Timer.getFPGATimestamp() >= start_time + timeout);
		else
			return (Timer.getFPGATimestamp() >= start_time + timeout) || reachedSetpoint;
	}

	@Override
	public void execute() {
		if(driveTrain.leftPID.onTarget() && driveTrain.rightPID.onTarget()) {
			reachedSetpoint = true;
		} else {
			reachedSetpoint = false;
		}
	}

	@Override
	public void end() {
		driveTrain.leftPID.disable();
		driveTrain.rightPID.disable();
		driveTrain.stop();
	}

	public void interrupt() {
		end();
	}
	
	@Override
	public void start() {
		start_time = Timer.getFPGATimestamp();
		
		robot.leftDriveEncoder.reset();
		robot.rightDriveEncoder.reset();
		
		leftEncoderStartDistance = robot.leftDriveEncoder.getDistance();
		rightEncoderStartDistance = robot.rightDriveEncoder.getDistance();
		
		driveTrain.leftPID.setOutputRange(-maxSpeed, maxSpeed);
		driveTrain.leftPID.setPID(P, I, D);
		driveTrain.leftPID.setSetpoint(distance + leftEncoderStartDistance);
		
		driveTrain.rightPID.setOutputRange(-maxSpeed, maxSpeed);
		driveTrain.rightPID.setPID(P, I, D);
		driveTrain.rightPID.setSetpoint(-(distance + rightEncoderStartDistance));
		
		driveTrain.leftPID.enable();
		driveTrain.rightPID.enable();
	}
	
	protected void interrupted() {
		end();
	}

}
