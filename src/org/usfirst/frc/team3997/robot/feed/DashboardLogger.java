package org.usfirst.frc.team3997.robot.feed;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.Ports;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardLogger {
	private RemoteControl humanControl;
	private RobotModel robot;
	

	public DashboardLogger(RobotModel robot, RemoteControl humanControl) {
		this.robot = robot;
		this.humanControl = humanControl;
		
		if(DriverStation.getInstance().isFMSAttached()) {
			putMatchInfo();
		}
	}

	public void updateData() {
		
		SmartDashboard.putNumber("DEBUG_FPGATimestamp", robot.getTimestamp());
		putRobotElectricalData();
		putJoystickAxesData();
		putMotorOutputs();
		putSensors();
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
	}

	
	
	
	
	public void putSensors() {
		SmartDashboard.putNumber("LEFT_ENC_DISTANCE", robot.leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("RIGHT_ENC_DISTANCE", robot.rightDriveEncoder.getDistance());
		SmartDashboard.putNumber("ARM_ANGLE", robot.getArmAngle());
		SmartDashboard.putNumber("GYRO", robot.getAngle());
		SmartDashboard.putNumber("DEBUG_GZ", robot.mpu_gyro.getRate());
	}
	
	

}
