package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.can.CANStatus;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;


import org.usfirst.frc.team3997.robot.hardware.Ports;
import org.usfirst.frc.team3997.robot.Params;
/*** Hardware (motors, pneumatics, sensors) for robot ***/
public class RobotModel extends Subsystem{
	/***Drive Motors***/
	public VictorSP leftDriveMotorA, leftDriveMotorB, rightDriveMotorA, rightDriveMotorB;
	/*** Arm Motors ***/
	public Spark leftArmMotor, rightArmMotor;
	/***Intake Motors***/
	public Spark leftIntakeMotor, rightIntakeMotor;
	/***Motor Groups for Drive***/
	public SpeedControllerGroup leftDriveMotors, rightDriveMotors;
	/***Motor Groups for Arm ***/
	public SpeedControllerGroup armMotors;
	/***Motor Groups for Intake ***/
	public SpeedControllerGroup intakeMotors;;
	/*** Drive Encoder ***/
	public Encoder leftDriveEncoder, rightDriveEncoder;
	/*** Arm Potentiometer ***/
	public TenTurnPotentiometer pot;
	/*** Compressor (not really needed) ***/
	public Compressor compressor;
	
	/***Intake solenoid (opens or closes claw)***/
	public DoubleSolenoid intakeSolenoid;
	/*** Wrist solenoid (brings up or down claw) **/
	public DoubleSolenoid wristSolenoid;
	/*** Gyro ***/
	public Gyro mpu_gyro;
	//public DigitalInput limitSwitch;
	

	/*** Timers for robot status***/
	public Timer timer, autoTimer, teleopTimer;
	/***Power Distribution Panel (Gives electricity to all motors)***/
	private PowerDistributionPanel pdp;
	
	CANStatus canStatus;
	/*
	 * TODO boolean enabled = c.enabled(); boolean pressureSwitch =
	 * c.getPressureSwitchValue(); double current = c.getCompressorCurrent();
	 */
	/*** Initalizes all hardware ***/
	public RobotModel() {
		pdp = new PowerDistributionPanel();
		// Pneumatics
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);
		//Solenoids for intake
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
		rightDriveMotors.setInverted(true); // negative value since wheels are inverted on one side
		// Init arm motors
		leftArmMotor = new Spark(Ports.LEFT_ARM_MOTOR_PWM_PORT);
		rightArmMotor = new Spark(Ports.RIGHT_ARM_MOTOR_PWM_PORT);
		// Invert right arm motor
		leftArmMotor.setInverted(true);
		rightArmMotor.setInverted(true);
		// Make a speed controller group for the arm
		armMotors = new SpeedControllerGroup(leftArmMotor, rightArmMotor);

		// Init intake motors
		leftIntakeMotor = new Spark(Ports.LEFT_INTAKE_MOTOR_PWM_PORT);
		rightIntakeMotor = new Spark(Ports.RIGHT_INTAKE_MOTOR_PWM_PORT);
		// Invert right intake motor
		leftIntakeMotor.setInverted(false);
		rightIntakeMotor.setInverted(false);
		
		//Speedcontroller group for intake motors
		intakeMotors = new SpeedControllerGroup(leftIntakeMotor, rightIntakeMotor);
		//Limit switch for intake to detect cube
		//limitSwitch = new DigitalInput(Ports.LIMIT_SWITCH);
		// TODO add real input channel

		//Initialize arm sensor
		AnalogInput.setGlobalSampleRate(62500);
		pot = new TenTurnPotentiometer(Ports.ARM_ENCODER);
		
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

		//TODO Probably Inverted twice
		leftDriveMotorA.setInverted(false);
		leftDriveMotorB.setInverted(false);
		rightDriveMotorA.setInverted(true);
		rightDriveMotorB.setInverted(true);


		timer = new Timer();
		timer.start();
		
		mpu_gyro = new MPU9250Gyro(Port.kOnboard);

		autoTimer = new Timer();
		// TODO add real url
		// camera.addServer("Server");

	}



	public enum Wheels {
		LeftWheels, RightWheels, AllWheels
	};

	/*** sets the speed for a given wheel(s) ***/
	public void setWheelSpeed(Wheels w, double speed) {
		switch (w) {
		case LeftWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			break;
		case RightWheels:
			rightDriveMotorA.set(speed); 
											
			rightDriveMotorB.set(speed); 
											
			break;
		case AllWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			rightDriveMotorA.set(speed); 					
			rightDriveMotorB.set(speed); 
			break;
		}
	}

	/*** returns the speed of a given wheel ***/
	public double getWheelSpeed(Wheels w) {
		switch (w) {
		case LeftWheels:
			return leftDriveMotorA.get();
		case RightWheels:
			return rightDriveMotorA.get();
		default:
			return 0;
		}
	}

	/*** resets variables and objects ***/
	public void reset() {
		resetEncoders();
		resetGyro();
	}


	/*** returns the voltage from PDP ***/
	public double getVoltage() {
		return pdp.getVoltage();
	}

	/*** returns the total energy of the PDP ***/
	public double getTotalEnergy() {
		return pdp.getTotalEnergy();
	}

	/*** returns the total current of the PDP ***/
	public double getTotalCurrent() {
		return pdp.getTotalCurrent();
	}

	/*** returns the total power of the PDP ***/
	public double getTotalPower() {
		return pdp.getTotalPower();
	}


	/*** resets the timer ***/
	public void resetTimer() {
		timer.reset();
	}

	/*** Gets current system clock ***/
	@SuppressWarnings("static-access")
	public double getTimestamp() {
		return timer.getFPGATimestamp();
	}

	/*** returns the time ***/
	public double getTime() {
		return timer.get();
	}

	/*** Resets encoders ***/
	public void resetEncoders() {
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();

	}
	/*** Returns difference between left and right encoder ***/
	public double getEncoderError() {
		return leftDriveEncoder.getDistance() - rightDriveEncoder.getDistance();
	}

	
	/*** Sets Left Drive output ***/
	public void setLeftMotors(double output) {
		leftDriveMotors.set(output);

	}
	/*** Sets Right Drive output ***/
	public void setRightMotors(double output) {
		rightDriveMotors.set(output);
	}
	
	/*** Sets arm output ***/
	public void moveArm(double speed) {
		armMotors.set(speed);
	}

	/*** Gets average arm output (-1 to 1) ***/
	public double getAverageArmSpeed() {
		return (leftArmMotor.getSpeed() + rightArmMotor.getSpeed()) / 2;
	}

	/*** Get pot raw value ***/
	public double getArmEncoderRawValue() {
		return pot.getValue();
		
	}
	/*** Get average pot raw value ***/
	public double getAverageArmEncoderRawValue() {
		return pot.getAverageValue();
	}
	/*** Get average pot voltage ***/
	public double getAverageArmVoltage() {
		return pot.getAverageVoltage();
	}
	/*** Get pot voltage ***/
	public double getArmVoltage() {
		return pot.getVoltage();
	}
	/*** Get arm angle ***/
	public double getArmAngle() {
		return pot.getAngle();
	}
	/*** set intae speed ***/
	public void intakeWheels(double speed) {
		intakeMotors.set(speed);

	}

	/*** Opens intake claw ***/
	public void openIntake() {
		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
	}
	/*** Closes Intake claw ***/
	public void closeIntake() {
		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	/*** Brings intake down ***/
	public void relaxWrist() {
		wristSolenoid.set(DoubleSolenoid.Value.kForward);
	}
	/*** Not used yet ***/
	public boolean getBlockTouching() {
		//return limitSwitch.get();
		return false;
	}
	/*** Intake block ***/
	public void intakeBlock() {
		intakeWheels(1);
	}
	
	/*** Outtake block ***/
	public void outtakeBlock() {
		intakeWheels(-1);
	}
	/*** Stops intake ***/
	public void stopIntake() {
		intakeWheels(0);
	}
	/*** Gets robot angle ***/
	public double getAngle() {
		return mpu_gyro.getAngle();
	}
	/*** Resets Gyro ***/
	public void resetGyro() {
		mpu_gyro.reset();
	}
	
	/*** Runs in loop ***/
	public void update() {
		mpu_gyro.getAngle();
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

	
}
