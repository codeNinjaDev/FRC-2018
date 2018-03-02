package org.usfirst.frc.team3997.robot.feed;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.Ports;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardLogger {
	private RemoteControl humanControl;
	private RobotModel robot;
	private Timer statTimer;
	
	private double max_velocity;
	private double max_acceleration;
	private double max_jerk;

	public DashboardLogger(RobotModel robot, RemoteControl humanControl) {
		this.robot = robot;
		this.humanControl = humanControl;
		statTimer = new Timer();
		statTimer.start();

	}

	public void updateData() {
		putDriverJoystickAxesData();
		putDriverMotorOutputs();
		putDriveEncoderData();
		putGamePadButtonPress();
		putGameData();
		putStats();

		SmartDashboard.putNumber("DEBUG_FPGATimestamp", robot.getTimestamp());
	}

	public void updateEssentialData() {
		SmartDashboard.putNumber("DRIVERJOY_driverLeftY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLY));
		SmartDashboard.putNumber("DRIVERJOY_driverRightX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRX));

		SmartDashboard.putNumber("LEFT_ENC_DISTANCE", robot.leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("RIGHT_ENC_DISTANCE", robot.rightDriveEncoder.getDistance());
		// putGamePadButtonPress();

	}

	public void putRobotCurrentData() {
		SmartDashboard.putNumber("PDP_Voltage", robot.getVoltage());
		SmartDashboard.putNumber("PDP_TotalEnergy", robot.getTotalEnergy());
		SmartDashboard.putNumber("PDP_TotalCurrent", robot.getTotalCurrent());
		SmartDashboard.putNumber("PDP_TotalPower", robot.getTotalPower());
	}

	public void putDriveMotorCurrentData() {
		SmartDashboard.putNumber("PDP_leftDriveMotorA", robot.getCurrent(Ports.LEFT_DRIVE_MOTOR_A_PDP_CHAN));
		SmartDashboard.putNumber("PDP_leftDriveMotorB", robot.getCurrent(Ports.LEFT_DRIVE_MOTOR_B_PDP_CHAN));
		SmartDashboard.putNumber("PDP_rightDriveMotorA", robot.getCurrent(Ports.RIGHT_DRIVE_MOTOR_A_PDP_CHAN));
		SmartDashboard.putNumber("PDP_rightDriveMotorB", robot.getCurrent(Ports.RIGHT_DRIVE_MOTOR_B_PDP_CHAN));
	}

	public void putDriveEncoderData() {
		SmartDashboard.putNumber("LEFT_ENC_DISTANCE", robot.leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("RIGHT_ENC_DISTANCE", robot.rightDriveEncoder.getDistance());
	}

	public void putDriverJoystickAxesData() {
		SmartDashboard.putNumber("DRIVERJOY_driverLeftX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLX));
		SmartDashboard.putNumber("DRIVERJOY_driverLeftY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLY));
		SmartDashboard.putNumber("DRIVERJOY_driverRightX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRX));
		SmartDashboard.putNumber("DRIVERJOY_driverRightY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRY));
	}

	public void putGamePadButtonPress() {
		SmartDashboard.putBoolean("ArcadeDrive", Params.USE_ARCADE_DRIVE);
		SmartDashboard.putBoolean("TankDrive", !Params.USE_ARCADE_DRIVE);
		SmartDashboard.putBoolean("BUTTON_slowDriveTier1Desired", humanControl.getSlowDriveTier1Desired());
		SmartDashboard.putBoolean("BUTTON_slowDriveTier2Desired", humanControl.getSlowDriveTier2Desired());

	}

	public void putDriverMotorOutputs() {
		SmartDashboard.putNumber("MOTOR_leftDriveMotorA", robot.leftDriveMotorA.get());
		SmartDashboard.putNumber("MOTOR_leftDriveMotorB", robot.leftDriveMotorB.get());
		SmartDashboard.putNumber("MOTOR_rightDriveMotorA", robot.rightDriveMotorA.get());
		SmartDashboard.putNumber("MOTOR_rightDriveMotorB", robot.rightDriveMotorB.get());
	}

	public void putGameData() {
		SmartDashboard.putString("Switch Colors", String.valueOf(PlateDetector.getSwitchColor()));
		SmartDashboard.putString("Scale Colors", String.valueOf(PlateDetector.getScaleColor()));

	}
	
	public void startTeleop() {
		max_velocity = 0;
		max_acceleration = 0;
		max_jerk = 0;
	}
	
	public void putStats() {
		double position = robot.leftDriveEncoder.getDistance();
		double velocity = position/statTimer.get();
		double acceleration = velocity/statTimer.get();
		double jerk = acceleration/statTimer.get();
		
		SmartDashboard.putNumber("Position", position);
		SmartDashboard.putNumber("Velocity", velocity);
		SmartDashboard.putNumber("Acceleration", acceleration);
		SmartDashboard.putNumber("Jerk", jerk);

		if(velocity > max_velocity) {
			max_velocity = velocity;
		} else if(acceleration > max_acceleration) {
			max_acceleration = acceleration;
		} else if(jerk > max_jerk) {
			max_jerk = jerk;
		}
		
		SmartDashboard.putNumber("Max Velocity", max_velocity);
		SmartDashboard.putNumber("Max Acceleration", max_acceleration);
		SmartDashboard.putNumber("Max Jerk", max_jerk);


	}

}
