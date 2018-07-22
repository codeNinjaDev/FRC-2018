package org.usfirst.frc.team3997.robot.feed;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardLogger {
	private RemoteControl humanControl;
	private RobotModel robot;
	private ArmController arm;
	private DriveController driveController;

	public DashboardLogger(RobotModel robot, RemoteControl humanControl, DriveController driveController, ArmController arm) {
		this.robot = robot;
		this.humanControl = humanControl;
		this.arm = arm;
		this.driveController = driveController;
		if(DriverStation.getInstance().isFMSAttached()) {
			putMatchInfo();
		}
	}

	public void updateData() {
		
		SmartDashboard.putNumber("DEBUG_FPGATimestamp", robot.getTimestamp());
		if(DriverStation.getInstance().isFMSAttached()) {
			putMatchInfo();
		}
		putRobotElectricalData();
		putJoystickAxesData();
		putMotorOutputs();
		putSensors();
		putPneumatics();
		if(DriverStation.getInstance().isAutonomous()) {
			SmartDashboard.putString("DS_MODE", "AUTONOMOUS");
		}  else if(DriverStation.getInstance().isOperatorControl()) {
			SmartDashboard.putString("DS_MODE", "TELEOP");
		} else {
			SmartDashboard.putString("DS_MODE", "DISABLED");
		}
		
		
	}


	public void putRobotElectricalData() {
		SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
		SmartDashboard.putBoolean("Brown Out", RobotController.isBrownedOut());
		SmartDashboard.putNumber("3Volt Rail Current", RobotController.getCurrent3V3());
		SmartDashboard.putNumber("5Volt Rail Current", RobotController.getCurrent5V());
		SmartDashboard.putNumber("6Volt Rail Current", RobotController.getCurrent6V());
		SmartDashboard.putNumber("Input Current", RobotController.getInputCurrent());
		SmartDashboard.putNumber("3Volt Rail Voltage", RobotController.getVoltage3V3());
		SmartDashboard.putNumber("5Volt Rail Voltage", RobotController.getVoltage5V());
		SmartDashboard.putNumber("6Volt Rail Voltage", RobotController.getVoltage6V());
		SmartDashboard.putNumber("Input Voltage", RobotController.getInputVoltage());
		SmartDashboard.putNumber("Compressor Current", robot.compressor.getCompressorCurrent());
		SmartDashboard.putBoolean("Pressure Switch Value", robot.compressor.getPressureSwitchValue());


	}

	public void putMatchInfo() {
		SmartDashboard.putString("EVENT_NAME", DriverStation.getInstance().getEventName());
		SmartDashboard.putNumber("MATCH_NUMBER", DriverStation.getInstance().getMatchNumber());
		SmartDashboard.putString("MATCH_TYPE", DriverStation.getInstance().getMatchType().toString());
		SmartDashboard.putString("ALLIANCE", DriverStation.getInstance().getAlliance().toString());
		SmartDashboard.putNumber("LOCATION", DriverStation.getInstance().getLocation());
		SmartDashboard.putString("GAME_DATA", DriverStation.getInstance().getGameSpecificMessage());
		SmartDashboard.putNumber("MATCH_TIME", DriverStation.getInstance().getMatchTime());

	}
	
	public void putJoystickAxesData() {
		SmartDashboard.putNumber("DRIVERJOY_operatorrLeftX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLX));
		SmartDashboard.putNumber("DRIVERJOY_driverLeftY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLY));
		SmartDashboard.putNumber("DRIVERJOY_driverRightX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRX));
		SmartDashboard.putNumber("DRIVERJOY_driverRightY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRY));
		SmartDashboard.putNumber("OPERATORJOY_operatorLeftX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy, RemoteControl.Axes.kLX));
		SmartDashboard.putNumber("OPERATORJOY_operatorLeftY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy, RemoteControl.Axes.kLY));
		SmartDashboard.putNumber("OPERATORJOY_operatorRightX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy, RemoteControl.Axes.kRX));
		SmartDashboard.putNumber("OPERATORJOY_operatorRightY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy, RemoteControl.Axes.kRY));
	}

	

	public void putMotorOutputs() {
		
		SmartDashboard.putBoolean("MOTOR_leftDriveMotorA_REVERSED", robot.leftDriveMotorA.getInverted());
		SmartDashboard.putBoolean("MOTOR_leftDriveMotorB_REVERSED", robot.leftDriveMotorB.getInverted());
		SmartDashboard.putBoolean("MOTOR_rightDriveMotorA_REVERSED", robot.leftDriveMotorA.getInverted());
		SmartDashboard.putBoolean("MOTOR_rightDriveMotorB_REVERSED", robot.rightDriveMotorB.getInverted());

		SmartDashboard.putNumber("MOTOR_leftDriveMotorA", robot.leftDriveMotorA.get());
		SmartDashboard.putNumber("MOTOR_leftDriveMotorB", robot.leftDriveMotorB.get());
		SmartDashboard.putNumber("MOTOR_rightDriveMotorA", robot.rightDriveMotorA.get());
		SmartDashboard.putNumber("MOTOR_rightDriveMotorB", robot.rightDriveMotorB.get());
		
		SmartDashboard.putNumber("LEFT_DRIVE_MOTORS", robot.leftDriveMotors.get());
		SmartDashboard.putNumber("RIGHT_DRIVE_MOTORS", robot.rightDriveMotors.get());

		SmartDashboard.putBoolean("MOTOR_leftIntakeMotor_REVERSED", robot.leftIntakeMotor.getInverted());
		SmartDashboard.putBoolean("MOTOR_rightIntakeMotor_REVERSED", robot.rightIntakeMotor.getInverted());

		SmartDashboard.putNumber("MOTOR_leftIntakeMotor", robot.leftIntakeMotor.get());
		SmartDashboard.putNumber("MOTOR_rightIntakeMotor", robot.rightIntakeMotor.get());

		SmartDashboard.putNumber("INTAKE_MOTORS", robot.intakeMotors.get());
		
		SmartDashboard.putBoolean("MOTOR_leftArmMotor", robot.leftArmMotor.getInverted());
		SmartDashboard.putBoolean("MOTOR_rightArmMotor", robot.rightArmMotor.getInverted());

		SmartDashboard.putNumber("LEFT_ARM_MOTOR", robot.leftArmMotor.get());
		SmartDashboard.putNumber("LEFT_ARM_MOTOR", robot.rightArmMotor.get());

		
		SmartDashboard.putNumber("ARM_MOTORS", robot.armMotors.get());
		SmartDashboard.putData("ARM_SENDABLE", robot.armMotors);
		SmartDashboard.putData("LEFT_DRIVE_SENDABLE", robot.leftDriveMotors);
		SmartDashboard.putData("RIGHT_DRIVE_SENDABLE", robot.rightDriveMotors);
		SmartDashboard.putData("INTAKE_WHEELS_SENDABLE", robot.intakeMotors);
	}

	
	public void putParamData() {
		SmartDashboard.putNumber("ARM_P", Params.arm_p);
		SmartDashboard.putNumber("ARM_I", Params.arm_i);
		SmartDashboard.putNumber("ARM_D", Params.arm_d);
		SmartDashboard.putNumber("DRIVE_P", Params.drive_p);
		SmartDashboard.putNumber("DRIVE_I", Params.drive_i);
		SmartDashboard.putNumber("DRIVE_D", Params.drive_d);
		SmartDashboard.putNumber("TURN_P", Params.turn_drive_p);		
		SmartDashboard.putNumber("TURN_I", Params.turn_drive_i);
		SmartDashboard.putNumber("TURN_D", Params.turn_drive_d);
		SmartDashboard.putNumber("ARM_SCALE_SETPOINT", Params.ARM_SCALE_SETPOINT);
		SmartDashboard.putNumber("ARM_SWITCH_SETPOINT", Params.ARM_SWITCH_SETPOINT);
		SmartDashboard.putNumber("ARM_FEED_SETPOINT", Params.ARM_FEED_SETPOINT);
		SmartDashboard.putNumber("MAX_SPEED", Params.MAX_SPEED);
		SmartDashboard.putNumber("DELTA_TIME_MP", Params.dt);
		SmartDashboard.putNumber("WHEEL_CIRCUMFERENCE", Params.WHEEL_CIRCUMFERENCE);
		SmartDashboard.putNumber("WHEEL_DIAMETER", Params.WHEEL_DIAMETER);
		SmartDashboard.putNumber("TRACK_BASE_WIDTH", Params.track_base_width);
		SmartDashboard.putNumber("WHEEL_BASE_WIDTH", Params.wheel_base_width);
		SmartDashboard.putNumber("X_SPEED_MULTIPLIER", Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER);
		SmartDashboard.putNumber("Y_SPEED_MULTIPLIER", Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER);
		



	}
	
	
	
	
	public void putSensors() {
		SmartDashboard.putNumber("LEFT_ENC_DISTANCE", robot.leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("RIGHT_ENC_DISTANCE", robot.rightDriveEncoder.getDistance());
		SmartDashboard.putNumber("LEFT_ENC_VELOCITY", robot.leftDriveEncoder.getRate());
		SmartDashboard.putNumber("RIGHT_ENC_VELOCITY", robot.rightDriveEncoder.getRate());
		
		SmartDashboard.putNumber("ACCELEROMETER_X", robot.getAccelX());
		SmartDashboard.putNumber("ACCELEROMETER_Y", robot.getAccelY());
		SmartDashboard.putNumber("ACCELEROMETER_Z", robot.getAccelZ());

		SmartDashboard.putNumber("ARM_ANGLE", robot.getArmAngle());
		SmartDashboard.putNumber("GYRO", robot.getAngle());
		SmartDashboard.putNumber("DEBUG_GZ", robot.mpu_gyro.getRate());
		SmartDashboard.putData("RIGHT_DRIVE_ENCODER_SENDABLE", robot.rightDriveEncoder);
		SmartDashboard.putData("LEFT_DRIVE_ENCODER_SENDABLE", robot.leftDriveEncoder);
		SmartDashboard.putData("GYRO_SENDABLE", robot.mpu_gyro);

	}
	
	
	public void putPneumatics() {
		SmartDashboard.putData("WRIST_PISTON_SENDABLE", robot.wristSolenoid);
		SmartDashboard.putData("Compressor_SENDABLE", robot.compressor);
		SmartDashboard.putData("INTAKE_PISTON_SENDABLE", robot.intakeSolenoid);
	}
}
