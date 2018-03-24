package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

import org.usfirst.frc.team3997.robot.hardware.Ports;
import org.usfirst.frc.team3997.robot.Params;

public class RobotModel {

	public VictorSP leftDriveMotorA, leftDriveMotorB, rightDriveMotorA, rightDriveMotorB;
	public Spark leftArmMotor, rightArmMotor, leftIntakeMotor, rightIntakeMotor;
	public SpeedControllerGroup leftDriveMotors, rightDriveMotors, armMotors, intakeMotors;
	public Encoder leftDriveEncoder, rightDriveEncoder;
	public AbsoluteEncoder armEncoder;
	public Compressor compressor;
	public DoubleSolenoid intakeSolenoid;
	public DoubleSolenoid wristSolenoid;

	//public DigitalInput limitSwitch;
	

	// public CameraServer camera;
	public Timer timer;
	public Timer autoTimer;
	public Timer teleopTimer;

	private PowerDistributionPanel pdp;
	private double leftDriveACurrent, leftDriveBCurrent, rightDriveACurrent, rightDriveBCurrent;

	/*
	 * TODO boolean enabled = c.enabled(); boolean pressureSwitch =
	 * c.getPressureSwitchValue(); double current = c.getCompressorCurrent();
	 */
	public RobotModel() {
		pdp = new PowerDistributionPanel();
		// Pneumatics
		compressor = new Compressor(Ports.COMPRESSOR_MODULE);
		intakeSolenoid = new DoubleSolenoid(Ports.INTAKE_SOLENOIDS[0], Ports.INTAKE_SOLENOIDS[1]);
		wristSolenoid = new DoubleSolenoid(Ports.WRIST_SOLENOIDS[0], Ports.WRIST_SOLENOIDS[1]);
		
		// Init drive motors
		leftDriveMotorA = new VictorSP(Ports.LEFT_DRIVE_MOTOR_A_PWM_PORT);
		leftDriveMotorB = new VictorSP(Ports.LEFT_DRIVE_MOTOR_B_PWM_PORT);
		rightDriveMotorA = new VictorSP(Ports.RIGHT_DRIVE_MOTOR_A_PWM_PORT);
		rightDriveMotorB = new VictorSP(Ports.RIGHT_DRIVE_MOTOR_B_PWM_PORT);
		// Make a Speed Controller group for Drive
		leftDriveMotors = new SpeedControllerGroup(leftDriveMotorA, leftDriveMotorB);
		rightDriveMotors = new SpeedControllerGroup(rightDriveMotorA, rightDriveMotorB);

		// Init arm motors
		leftArmMotor = new Spark(Ports.LEFT_ARM_MOTOR_PWM_PORT);
		rightArmMotor = new Spark(Ports.RIGHT_ARM_MOTOR_PWM_PORT);
		// Invert right arm motor
		leftArmMotor.setInverted(false);
		rightArmMotor.setInverted(true);
		// Make a speed controller group for the arm
		armMotors = new SpeedControllerGroup(leftArmMotor, rightArmMotor);

		// Init intake motors
		leftIntakeMotor = new Spark(Ports.LEFT_INTAKE_MOTOR_PWM_PORT);
		rightIntakeMotor = new Spark(Ports.RIGHT_INTAKE_MOTOR_PWM_PORT);
		// Invert right intake motor
		leftIntakeMotor.setInverted(false);
		rightIntakeMotor.setInverted(true);
		
		//Limit switch for intake to detect cube
		//limitSwitch = new DigitalInput(Ports.LIMIT_SWITCH);
		// TODO add real input channel

		//Initialize arm sensor
		AnalogInput.setGlobalSampleRate(62500);
		armEncoder = new AbsoluteEncoder(Ports.ARM_ENCODER);

		//Initialize drive encoders
		leftDriveEncoder = new Encoder(Ports.LEFT_DRIVE_ENCODER_PORTS[0], Ports.LEFT_DRIVE_ENCODER_PORTS[1]);
		rightDriveEncoder = new Encoder(Ports.RIGHT_DRIVE_ENCODER_PORTS[0], Ports.RIGHT_DRIVE_ENCODER_PORTS[1]);

		//Encoder setup
		leftDriveEncoder.setReverseDirection(false);
		leftDriveEncoder.setDistancePerPulse(((1.0) / Params.PULSES_PER_ROTATION) * (Params.WHEEL_CIRCUMFERENCE));
		leftDriveEncoder.setSamplesToAverage(1);
		rightDriveEncoder.setReverseDirection(false);
		rightDriveEncoder.setDistancePerPulse(((1.0) / Params.PULSES_PER_ROTATION) * (Params.WHEEL_CIRCUMFERENCE));
		rightDriveEncoder.setSamplesToAverage(1);

		leftDriveMotorA.setSafetyEnabled(false);
		leftDriveMotorB.setSafetyEnabled(false);
		rightDriveMotorA.setSafetyEnabled(false);
		rightDriveMotorB.setSafetyEnabled(false);

		leftDriveMotorA.setInverted(false);
		leftDriveMotorB.setInverted(false);
		rightDriveMotorA.setInverted(true);
		rightDriveMotorB.setInverted(true);

		leftDriveACurrent = 0;
		leftDriveBCurrent = 0;
		rightDriveACurrent = 0;
		rightDriveBCurrent = 0;

		timer = new Timer();
		timer.start();

		// TODO add real url
		// camera.addServer("Server");

	}

	

	public enum Wheels {
		LeftWheels, RightWheels, AllWheels
	};

	// sets the speed for a given wheel(s)
	public void setWheelSpeed(Wheels w, double speed) {
		switch (w) {
		case LeftWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			break;
		case RightWheels:
			rightDriveMotorA.set(-speed); // negative value since wheels are
											// inverted on robot
			rightDriveMotorB.set(-speed); // negative value since wheels are
											// inverted on robot
			break;
		case AllWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			rightDriveMotorA.set(-speed); // negative value since wheels are
											// inverted on robot
			rightDriveMotorB.set(-speed); // negative value since wheels are
											// inverted on robot
			break;
		}
	}

	// returns the speed of a given wheel
	public double getWheelSpeed(Wheels w) {
		switch (w) {
		case LeftWheels:
			return leftDriveMotorA.get();
		case RightWheels:
			return -rightDriveMotorA.get();
		default:
			return 0;
		}
	}

	// resets variables and objects
	public void reset() {
		resetEncoders();
	}

	// initializes variables pertaining to current
	public void updateCurrent() {
		leftDriveACurrent = pdp.getCurrent(Ports.LEFT_DRIVE_MOTOR_A_PDP_CHAN);
		leftDriveBCurrent = pdp.getCurrent(Ports.LEFT_DRIVE_MOTOR_B_PDP_CHAN);
		rightDriveACurrent = pdp.getCurrent(Ports.RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
		rightDriveBCurrent = pdp.getCurrent(Ports.RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	}

	// returns the voltage
	public double getVoltage() {
		return pdp.getVoltage();
	}

	// returns the total energy of the PDP
	public double getTotalEnergy() {
		return pdp.getTotalEnergy();
	}

	// returns the total current of the PDP
	public double getTotalCurrent() {
		return pdp.getTotalCurrent();
	}

	// returns the total power of the PDP
	public double getTotalPower() {
		return pdp.getTotalPower();
	}

	// returns the current of a given channel
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

	// resets the timer
	public void resetTimer() {
		timer.reset();
	}

	public double getTimestamp() {
		return timer.getFPGATimestamp();
	}

	// returns the time
	public double getTime() {
		return timer.get();
	}

	// encoders
	public void resetEncoders() {
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();

	}

	public double getEncoderError() {
		return leftDriveEncoder.getDistance() - rightDriveEncoder.getDistance();
	}

	
	public void setLeftMotors(double output) {
		leftDriveMotorA.set(output);
		leftDriveMotorB.set(output);

	}

	public void setRightMotors(double output) {
		rightDriveMotorA.set(output);
		rightDriveMotorB.set(output);
	}

	public void moveArm(double speed) {
		armMotors.set(speed);
	}

	public double getAverageArmSpeed() {
		return (leftArmMotor.getSpeed() + rightArmMotor.getSpeed()) / 2;
	}

	public double getArmEncoderRawValue() {
		return armEncoder.getValue();
		
	}

	public double getAverageArmEncoderRawValue() {
		return armEncoder.getAverageValue();
	}

	public double getAverageArmVoltage() {
		return armEncoder.getAverageVoltage();
	}

	public double getArmVoltage() {
		return armEncoder.getVoltage();
	}

	public double getArmAngle() {
		return armEncoder.getAngle();
	}

	public void intakeWheels(double speed) {
		intakeMotors.set(speed);

	}

	public void openIntake() {
		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void closeIntake() {
		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
	}
	
	public void flexWrist() {
		wristSolenoid.set(DoubleSolenoid.Value.kForward);
	}
	
	public void relaxWrist() {
		wristSolenoid.set(DoubleSolenoid.Value.kReverse);
	}
	
	public boolean getBlockTouching() {
		//return limitSwitch.get();
		return false;
	}
	public void intakeBlock() {
		intakeWheels(-.5);
	}
	

	public void outtakeBlock() {
		intakeWheels(0.3);
	}
	public void stopIntake() {
		intakeWheels(0);
	}
	
	
	public void setVoltage(double desiredVoltage) {
	    
	    double batteryVoltage = pdp.getVoltage();

	    if (desiredVoltage > batteryVoltage) {
	        leftDriveMotors.set(1.0);
	        rightDriveMotors.set(1.0);

	    }
	    else if (desiredVoltage < -1*batteryVoltage) {
	    	leftDriveMotors.set(-1.0);
	        rightDriveMotors.set(-1.0);	    }
	    else {
	    	leftDriveMotors.set(desiredVoltage/batteryVoltage);
	        rightDriveMotors.set(desiredVoltage/batteryVoltage);
	    }

	}
}
