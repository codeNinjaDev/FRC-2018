package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.Ports;

/**
 * Contains all the hardware for the robot, including motors, sensors, and
 * power.
 * 
 * @category hardware
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 *
 */
public class RobotModel {

	public VictorSP leftDriveMotorA, leftDriveMotorB, rightDriveMotorA, rightDriveMotorB;
	public SpeedControllerGroup leftDriveMotors, rightDriveMotors;
	public Encoder leftDriveEncoder, rightDriveEncoder;
	public MPU9250Gyro gyro;

	// public CameraServer camera;
	public Timer timer;

	private PowerDistributionPanel pdp;
	private double leftDriveACurrent, leftDriveBCurrent, rightDriveACurrent, rightDriveBCurrent;

	/**
	 * Initializes all the hardware variables
	 * 
	 * 
	 * 
	 */
	public RobotModel() {
		pdp = new PowerDistributionPanel();
		// Init drive motors
		leftDriveMotorA = new VictorSP(Ports.LEFT_DRIVE_MOTOR_A_PWM_PORT);
		leftDriveMotorB = new VictorSP(Ports.LEFT_DRIVE_MOTOR_B_PWM_PORT);
		rightDriveMotorA = new VictorSP(Ports.RIGHT_DRIVE_MOTOR_A_PWM_PORT);
		rightDriveMotorB = new VictorSP(Ports.RIGHT_DRIVE_MOTOR_B_PWM_PORT);

		leftDriveMotors = new SpeedControllerGroup(leftDriveMotorA, leftDriveMotorB);
		rightDriveMotors = new SpeedControllerGroup(rightDriveMotorA, rightDriveMotorB);

		// TODO add real input channel
		// gyro = new AnalogGyro(channel);

		leftDriveEncoder = new Encoder(Ports.LEFT_DRIVE_ENCODER_PORTS[0], Ports.LEFT_DRIVE_ENCODER_PORTS[1]);
		rightDriveEncoder = new Encoder(Ports.RIGHT_DRIVE_ENCODER_PORTS[0], Ports.RIGHT_DRIVE_ENCODER_PORTS[1]);

		leftDriveEncoder.setReverseDirection(false);
		//Distance Per Encoder Pulse - how far it goes for one tick of encoder
		leftDriveEncoder.setDistancePerPulse(Params.WHEEL_CIRCUMFERENCE / Params.PULSES_PER_ROTATION);
		leftDriveEncoder.setSamplesToAverage(1);
		rightDriveEncoder.setReverseDirection(false);
		//Distance Per Encoder Pulse - how far it goes for one tick of encoder
		rightDriveEncoder.setDistancePerPulse(Params.WHEEL_CIRCUMFERENCE / Params.PULSES_PER_ROTATION);
		rightDriveEncoder.setSamplesToAverage(1);

		leftDriveMotorA.setSafetyEnabled(false);
		leftDriveMotorB.setSafetyEnabled(false);
		rightDriveMotorA.setSafetyEnabled(false);
		rightDriveMotorB.setSafetyEnabled(false);

		leftDriveMotorA.setInverted(false);
		leftDriveMotorB.setInverted(false);
		rightDriveMotorA.setInverted(false);
		rightDriveMotorB.setInverted(false);

		leftDriveACurrent = 0;
		leftDriveBCurrent = 0;
		rightDriveACurrent = 0;
		rightDriveBCurrent = 0;

		timer = new Timer();
		timer.start();

		gyro = new MPU9250Gyro(Port.kOnboard);
	}
	/**
	 * Updates Gyro yaw interface
	 **/
	public void updateGyro() {
		//gyro.update();
	}

	/**
	 * Enum of Wheels
	 **/
	public enum Wheels {
		LeftWheels, RightWheels, AllWheels
	};

	/**
	 * Sets the speed for a given wheel(s)
	 * 
	 * @param w
	 *            Which wheels? (Wheels enum)
	 * @param speed
	 *            What speed? (-1 to 1)
	 *
	 **/
	public void setWheelSpeed(Wheels w, double speed) {
		switch (w) {
		case LeftWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			break;
		case RightWheels:
			// TODO figure out why in the world this works while setRightMotors() works
			rightDriveMotorA.set(-speed); // negative value since wheels are
											// inverted on robot
			rightDriveMotorB.set(-speed); // negative value since wheels are
											// inverted on robot
			break;
		case AllWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			// TODO figure out why in the world this works while setRightMotors() works

			rightDriveMotorA.set(-speed); // negative value since wheels are
											// inverted on robot
			rightDriveMotorB.set(-speed); // negative value since wheels are
											// inverted on robot
			break;
		}
	}

	/***
	 * Returns the speed of a given wheel
	 * 
	 * @param w
	 *            Which wheels? (Wheels enum)
	 **/
	public double getWheelSpeed(Wheels w) {
		switch (w) {
		case LeftWheels:
			return leftDriveMotorA.get();
		case RightWheels:
			// TODO figure out why in the world this works while setRightMotors() works
			return -rightDriveMotorA.get();
		default:
			return 0;
		}
	}

	/**
	 * resets hardware <i>e.g encoders, gyros</i>
	 * 
	 * 
	 **/
	public void reset() {
		resetEncoders();
		gyro.reset();
	}

	/** Initializes variables pertaining to current **/
	public void updateCurrent() {
		leftDriveACurrent = pdp.getCurrent(Ports.LEFT_DRIVE_MOTOR_A_PDP_CHAN);
		leftDriveBCurrent = pdp.getCurrent(Ports.LEFT_DRIVE_MOTOR_B_PDP_CHAN);
		rightDriveACurrent = pdp.getCurrent(Ports.RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
		rightDriveBCurrent = pdp.getCurrent(Ports.RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	}

	/** Returns the voltage **/
	public double getVoltage() {
		return pdp.getVoltage();
	}

	/** Returns the total energy of the PDP ({@link PowerDistributionPanel}) **/
	public double getTotalEnergy() {
		return pdp.getTotalEnergy();
	}

	/** Returns the total current of the PDP ({@link PowerDistributionPanel}) **/
	public double getTotalCurrent() {
		return pdp.getTotalCurrent();
	}

	/** Returns the total power of the PDP ({@link PowerDistributionPanel}) **/
	public double getTotalPower() {
		return pdp.getTotalPower();
	}

	/** Returns the current of a given channel ({@link PowerDistributionPanel}) **/
	public double getCurrent(int channel) {
		switch (channel) {
		case Ports.RIGHT_DRIVE_MOTOR_A_PDP_CHAN:
			return rightDriveACurrent;
		case Ports.RIGHT_DRIVE_MOTOR_B_PDP_CHAN:
			return rightDriveBCurrent;
		case Ports.LEFT_DRIVE_MOTOR_A_PDP_CHAN:
			return leftDriveACurrent;
		case Ports.LEFT_DRIVE_MOTOR_B_PDP_CHAN:
			return leftDriveBCurrent;
		default:
			return -1;
		}
	}

	/** Resets the timer **/
	public void resetTimer() {
		timer.reset();
	}

	/** Gets Roborio timestamp **/
	public double getTimestamp() {
		return timer.getFPGATimestamp();
	}

	/** Gets the timer time **/
	public double getTime() {
		return timer.get();
	}

	/** Resets econder values **/
	public void resetEncoders() {
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
	}

	/** Gets the difference between left and right {@link Encoder}s **/
	public double getEncoderError() {
		return leftDriveEncoder.getDistance() - rightDriveEncoder.getDistance();
	}

	/** Resets gyro yaw value **/
	public void resetGyro() {
		//gyro.reset();
	}

	/** Gets gyro yaw value **/
	public double getAngle() {
		//return gyro.getAngle();
		return 0;
	}

	/**
	 * Sets the speed of the left motors
	 * 
	 * @param output
	 *            speed of motor (-1 to 1)
	 **/
	public void setLeftMotors(double output) {
		leftDriveMotorA.set(output);
		leftDriveMotorB.set(output);

	}

	/**
	 * Sets the speed of the right motors
	 * 
	 * @param output
	 *            speed of motor (-1 to 1)
	 **/

	public void setRightMotors(double output) {
		// negative value since wheels are
		// inverted on robot
		rightDriveMotorA.set(-output);
		rightDriveMotorB.set(-output);
	}

}
