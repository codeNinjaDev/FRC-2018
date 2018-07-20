package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Drives Left and Right Side PID independently ***/
public class DriveDistanceAction extends Command{

	private DriveController driveTrain;
	private RobotModel robot;
	
	/*** Distance Setpoint ***/
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
	/*** Drives Forward with independent left and right PID controllers
	 * 
	 * @param controllers ALl classes that control robot functionality
	 * @param distance Desired distance setpoint
	 * @param maxSpeed Max output of PID controller
	 * @param timeout Allowed time for action to take place
	 * @param waitForTimeout Whether to wait the full timeout, even if the setpoint is reached
	 */
	public DriveDistanceAction(double distance, double maxSpeed, double timeout, boolean waitForTimeout) {
		requires(Robot.driveController);
		requires(Robot.robot);
		
		this.driveTrain = Robot.driveController;
		this.distance = distance;
		this.timeout = timeout;
		this.robot = Robot.robot;
		this.maxSpeed = maxSpeed;
		this.waitForTimeout = waitForTimeout;
		
		reachedSetpoint = false;
		leftEncoderStartDistance = 0.0; 
		rightEncoderStartDistance = 0.0;
		
		P = .02;
		// TODO Might Need to change pID values
		I = Params.drive_i;
		D = 0.1;
		
		SmartDashboard.putNumber("DRIVE_PID_P", P);
		SmartDashboard.putNumber("DRIVE_PID_I", I);
		SmartDashboard.putNumber("DRIVE_PID_D", D);

	}
	@Override
	/*** Checks if action is finished ***/
	public boolean isFinished() {
		//Check if we want to only wait for Timeout
		if(waitForTimeout) 
			return (Timer.getFPGATimestamp() >= start_time + timeout);
		else //If not, end if we reached the setpoint
			return (Timer.getFPGATimestamp() >= start_time + timeout) || reachedSetpoint;
	}

	@Override
	/*** Updates action code in a loop ***/
	public void execute() {
		//Checks if pid controllers are reached setpoint
		if(driveTrain.leftPID.onTarget() && driveTrain.rightPID.onTarget()) {
			reachedSetpoint = true;
		} else {
			reachedSetpoint = false;
		}
	}

	@Override
	/*** Code that runs when action ends ***/
	public void end() {
		//Disables PID and stops drive
		driveTrain.leftPID.disable();
		driveTrain.rightPID.disable();
		driveTrain.stop();
	}

	@Override
	/*** Starts action ***/
	protected void initialize() {
		//Starts Timer
		start_time = Timer.getFPGATimestamp();
		//Configures encoders to measuring magnitude, not rate
		robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		//Resets encoders
		robot.leftDriveEncoder.reset();
		robot.rightDriveEncoder.reset();
		
		leftEncoderStartDistance = robot.leftDriveEncoder.getDistance();
		rightEncoderStartDistance = robot.rightDriveEncoder.getDistance();
		
		//Sets PID Outputrange, constants, and setpoints
		
		driveTrain.leftPID.setOutputRange(-maxSpeed, maxSpeed);
		driveTrain.leftPID.setPID(P, I, D);
		driveTrain.leftPID.setSetpoint(distance);
		
		driveTrain.rightPID.setOutputRange(-maxSpeed, maxSpeed);
		driveTrain.rightPID.setPID(P, I, D);
		driveTrain.rightPID.setSetpoint(distance);
		
		//Starts PID
		driveTrain.leftPID.enable();
		driveTrain.rightPID.enable();
	}
	
	protected void interrupted() {
		end();
	}
}
