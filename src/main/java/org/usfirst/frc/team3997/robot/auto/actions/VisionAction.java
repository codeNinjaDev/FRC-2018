package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.controllers.VisionController;
import org.usfirst.frc.team3997.robot.hardware.RobotHardware;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionAction extends Command{
	private DriveController driveTrain;
	private RobotHardware robot;
	private VisionController vision;
	
	private int setpoint;
	private double distance;
	private double timeout, start_time;
	private double maxSpeed;
	private double P, I, D;
	private double leftEncoderStartDistance, rightEncoderStartDistance;
	
	private boolean reachedSetpoint, waitForTimeout;
	
	public VisionAction(int setpoint, double maxSpeed, double timeout, boolean waitForTimeout) {
		requires(Robot.driveController);
		requires(Robot.visionController);
		requires(Robot.robot);


		this.driveTrain = Robot.driveController;
		this.vision = Robot.visionController;
		this.setpoint = setpoint;
		this.timeout = timeout;
		this.robot = Robot.robot;
		this.maxSpeed = maxSpeed;
		this.waitForTimeout = waitForTimeout;
		
		reachedSetpoint = false;
		leftEncoderStartDistance = 0.0; 
		rightEncoderStartDistance = 0.0;
		
		P = 0.3;
		//TODO Might Need to change pID values
		I = 0.0;
		D = 0.0;
		
		SmartDashboard.putNumber("DRIVE_PID_P", P);
		SmartDashboard.putNumber("DRIVE_PID_I", I);
		SmartDashboard.putNumber("DRIVE_PID_D", D);

	}
	@Override
	public boolean isFinished() {
		if(waitForTimeout) 
			return (Timer.getFPGATimestamp() >= start_time + timeout);
		else
			return (Timer.getFPGATimestamp() >= start_time + timeout);// ||(driveTrain.visionPID.onTarget());
	}

	@Override
	public void execute() {

		/*
		 * if(driveTrain.visionPID.getRightContour() == 0.0) {
		 * 
		 * 		driveTrain.visionPID.disable();
		 * 		SmartDashboard.putString("Running_VISION", "STOPPED");
		 * } else {
		 * 
		 * 		driveTrain.visionPID.enable();
		 * 		SmartDashboard.putString("Running_VISION", "Running");
		 * }
		 */
	}

	@Override
	public void end() {
		//driveTrain.visionPID.disable();
		
		driveTrain.stop();
		SmartDashboard.putString("Running_VISION", "DONE");
	}

	@Override
	public void initialize() {
		start_time = Timer.getFPGATimestamp();
		
		robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);

		robot.leftDriveEncoder.reset();
		robot.rightDriveEncoder.reset();
		
		leftEncoderStartDistance = robot.leftDriveEncoder.getDistance();
		leftEncoderStartDistance = robot.rightDriveEncoder.getDistance();
		
		/*
		 * driveTrain.visionPID.setOutputRange(-maxSpeed, maxSpeed);
		 * SmartDashboard.putNumer("Vision_P", P);
		 *  SmartDashboard.putNumer("Vision_I", I);
		 *  SmartDashboard.putNumer("Vision_D", D);
		 * 
		 * driveTrain.visionPID.setPID(P, I, D);
		 * driveTrain.visionPID.setSetpoint(setpoint + leftEncoderStartDistance);
		 * 
		 * driveTrain.visionPID.enable();
		 * */
	}
	protected void interrupted() {
		end();
	}
}
