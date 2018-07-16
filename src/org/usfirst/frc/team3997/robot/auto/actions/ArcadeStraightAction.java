package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Drive Forward action that uses arcade drive **/
public class ArcadeStraightAction extends Command {
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
	private double afterSetpointTime, timeAfterHit, start_time;
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
	 * @param controllers Gets all robot systems
	 * @param distance Gets desired distance setpoint
	 * @param maxSpeed Sets max speed during action
	 * @param timeout How much time allowed to reach setpoint
	 * @param timeAfterHit How much cushion time after reaching setpoint before ending action (Doesn't do anything currently)
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

		P = .02;
		// TODO Might Need to change pID values
		I = Params.drive_i;
		D =  .1;

		SmartDashboard.putNumber("DRIVE_PID_P", P);
		SmartDashboard.putNumber("DRIVE_PID_I", I);
		SmartDashboard.putNumber("DRIVE_PID_D", D);
		
		requires(driveTrain);

	}

	/*** Checks if timeout is finished or reached setpoint ***/
	protected boolean isFinished() {
		//Checks if current time is greater than timeout 
		return (Timer.getFPGATimestamp() >= start_time + timeout) || reachedSetpoint;

	}

	/*** Checks if PID is complete ***/
	protected void execute() {
		if (driveTrain.straightPID.onTarget()) {
			reachedSetpoint = true;
		} else {
			reachedSetpoint = false;
		}
	}

	/*** Disables PID ***/
	protected void end() {
		driveTrain.leftPID.disable();
		driveTrain.rightPID.disable();
		driveTrain.straightPID.disable();
	}

	/*** Starts timer and PID ***/
	protected void initialize() {
		//Starts the timer
		start_time = Timer.getFPGATimestamp();
		
		//Configures encoders to measuring magnitude, not rate
		robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		//Reset Encoders before startng
		robot.leftDriveEncoder.reset();
		robot.rightDriveEncoder.reset();

		leftEncoderStartDistance = robot.leftDriveEncoder.getDistance();
		leftEncoderStartDistance = robot.rightDriveEncoder.getDistance();
		//Set Output range and PID Constants
		driveTrain.straightPID.setOutputRange(-maxSpeed, maxSpeed);
		driveTrain.straightPID.setPID(P, I, D);
		// TODO Maybe distance - encoderStartDistance
		driveTrain.straightPID.setSetpoint(distance);
		//Starts arcade PID staright
		driveTrain.straightPID.enable();
	}

}
