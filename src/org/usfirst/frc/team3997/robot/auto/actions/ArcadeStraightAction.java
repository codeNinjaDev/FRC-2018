package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Drive Forward action that uses arcade drive **/
public class ArcadeStraightAction extends Action {
	private DriveController driveTrain;
	private RobotModel robot;
	/*** Instance variable for desired distance ***/
	private double distance;
	/*** Instance variable or allowed timeout ***/
	private double timeout;
	/*** Instance variable for max speed set***/
	private double maxSpeed;
	/*** Instance variables for P,I,D of straight PID controller ***/
	private double P, I, D;
	/*** Residual distance of left and right encoder ***/
	private double leftEncoderStartDistance, rightEncoderStartDistance;
	//Not sure
	private double afterSetpointTime, timeAfterHit;
	/*** Boolean checks if reached distance ***/
	private boolean reachedSetpoint;
	//Not sure
	private int target_pass;

	/*** <h1> ArcadeStraightAction Constructor </h1>
	 * 
	 * <h2> Activity: </h2>
	 * <ul>
	 * 	<li>Initializes DriveController and RobotModel object from MasterController</li>
	 * <li>Initializes distance, timeout, maxSpeed from Constructor params</li>
	 * 
	 * 
	 * 
	 * </ul>
	 * @param controllers
	 * @param distance
	 * @param maxSpeed
	 * @param timeout
	 * @param timeAfterHit
	 * 
	 */
	public ArcadeStraightAction(MasterController controllers, double distance, double maxSpeed, double timeout,
			double timeAfterHit) {
		this.driveTrain = controllers.getDriveController();
		this.distance = distance;
		this.timeout = timeout;
		this.robot = controllers.getRobotModel();
		this.maxSpeed = maxSpeed;
		this.timeAfterHit = timeAfterHit;
		start_time = 0;
		reachedSetpoint = false;
		leftEncoderStartDistance = 0.0;
		rightEncoderStartDistance = 0.0;

		P = Params.drive_p;
		// TODO Might Need to change pID values
		I = Params.drive_i;
		D =  Params.drive_d;

		SmartDashboard.putNumber("DRIVE_PID_P", P);
		SmartDashboard.putNumber("DRIVE_PID_I", I);
		SmartDashboard.putNumber("DRIVE_PID_D", D);

	}

	@Override
	/*** Checks if timeout is finished or reached setpoint ***/
	public boolean isFinished() {

		return (Timer.getFPGATimestamp() >= start_time + timeout) || reachedSetpoint;

	}

	@Override
	/*** Updates gyro and checks if PID is complete ***/
	public void update() {
		if (driveTrain.leftPID.onTarget() && driveTrain.rightPID.onTarget()) {
			reachedSetpoint = true;
		} else {
			reachedSetpoint = false;
		}
	}

	@Override
	/*** Disables PID ***/
	public void finish() {
		driveTrain.leftPID.disable();
		driveTrain.rightPID.disable();
		driveTrain.straightPID.disable();
	}

	@Override
	/*** Starts timer and PID ***/
	public void start() {
		start_time = Timer.getFPGATimestamp();

		robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);

		robot.leftDriveEncoder.reset();
		robot.rightDriveEncoder.reset();

		leftEncoderStartDistance = robot.leftDriveEncoder.getDistance();
		leftEncoderStartDistance = robot.rightDriveEncoder.getDistance();

		driveTrain.straightPID.setOutputRange(-maxSpeed, maxSpeed);
		driveTrain.straightPID.setPID(P, I, D);
		// TODO Maybe distance + encoderStartDistance or - encoder
		driveTrain.straightPID.setSetpoint(distance);

		driveTrain.straightPID.enable();
	}

}
