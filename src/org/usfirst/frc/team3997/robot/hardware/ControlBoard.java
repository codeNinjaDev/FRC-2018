
package org.usfirst.frc.team3997.robot.hardware;

import org.usfirst.frc.team3997.robot.Params;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/** Handles inputs from controller 
 * @category hardware
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 * 
 * **/
public class ControlBoard extends RemoteControl {
	/** Driver Buttons **/
	public ButtonReader driveBackButton, driveBackOtherButton;
	//Operator Buttons
/** Operator Buttons **/
	public ButtonReader armSwitchButton, armScaleButton, armFeedButton, intakeButton, outtakeButton;
	/*** Arm Override Trigger ***/
	public ToggleButtonReader armManualButton;
/** Driver Triggers **/
	public TriggerReader slowDriveTier1Button, slowDriveTier2Button, outtakeWheelsButton;
	/*** Booleans for relax wrist TODO for offseason streamline ***/
	private boolean slowDriveTier1Desired, slowDriveTier2Desired,
			driveBackDesired, driveBackOtherDesired, toggleArmManualDesired, armSwitchDesired, armScaleDesired, armFeedDesired, intakeDesired, outtakeDesired, armShifterDesired, outtakeWheelsDesired;

	/** Driver joystick axes **/
	private double driverLeftJoyX, driverLeftJoyY, driverRightJoyX, driverRightJoyY;
	/** Operator joystick axes **/
	private double operatorLeftJoyX, operatorLeftJoyY, operatorRightJoyX, operatorRightJoyY;
	/** Joystick **/
	private Joystick driverJoy, operatorJoy;
	private boolean intakeManualDesired;

	/** Initializes all controller inputs **/
	public ControlBoard() {
		driverJoy = new Joystick(Ports.DRIVER_JOY_USB_PORT);
		operatorJoy = new Joystick(Ports.OPERATOR_JOY_USB_PORT);

		if (Ports.USING_WIN_DRIVER_STATION) {
			//Driver Controls
			
			driveBackButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_LEFT_BUMPER, "DRIVE_BACK");
			driveBackOtherButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_RIGHT_BUMPER, "DRIVE_OTHER_BACK");
			
			slowDriveTier1Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS, "BRAKE_1");
			slowDriveTier2Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_LEFT_TRIGGER_AXIS, "BRAKE_2");
			
			//Operator Controls
			armScaleButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_YELLOW_BUTTON, "SCALE");
			armSwitchButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_GREEN_BUTTON, "SWITCH");
			armFeedButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_BLUE_BUTTON, "FEED");
			armManualButton = new ToggleButtonReader(operatorJoy, XInput.XINPUT_WIN_BACK_BUTTON, "ARM_MANUAL");
			
			
			outtakeWheelsButton = new TriggerReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS, "OUT_WHEELS");
			intakeButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_BUMPER, "INTAKE");
			outtakeButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_LEFT_BUMPER, "OUTTAKE");
			
			
		}

		driverLeftJoyX = 0;
		driverLeftJoyY = 0;
		driverRightJoyX = 0;
		driverRightJoyY = 0;

		// Driver variableS		
		slowDriveTier1Desired = false;
		slowDriveTier2Desired = false;
		driveBackDesired = false;
		driveBackOtherDesired = false;
		
		//Operator Vars
		armSwitchDesired = false;
		armScaleDesired = false;
		armFeedDesired = false;
		toggleArmManualDesired = false;
		
		intakeDesired = false;
		outtakeDesired = false;
		outtakeWheelsDesired = false;
	}
	/** Reads all controller inputs **/
	public void readControls() {
		readAllButtons();
		if (Ports.USING_WIN_DRIVER_STATION) {
			driverLeftJoyX = driverJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_X_AXIS);
			driverLeftJoyY = -driverJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_Y_AXIS);
			driverRightJoyX = driverJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_X_AXIS);
			driverRightJoyY = -driverJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_Y_AXIS);

			operatorLeftJoyX = operatorJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_X_AXIS);
			operatorLeftJoyY = -operatorJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_Y_AXIS);
			operatorRightJoyX = operatorJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_X_AXIS);
			operatorRightJoyY = -operatorJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_Y_AXIS);
		}

		// Driver Variables

		slowDriveTier1Desired = slowDriveTier1Button.isDown();
		slowDriveTier2Desired = slowDriveTier2Button.isDown();
		driveBackDesired = driveBackButton.isDown();
		driveBackOtherDesired = driveBackOtherButton.isDown();
		
		//Operator Vars
		armSwitchDesired = armSwitchButton.isDown();
		armScaleDesired = armScaleButton.isDown();
		armFeedDesired = armFeedButton.isDown();
		toggleArmManualDesired = armManualButton.getState();
		intakeDesired = intakeButton.isDown();
		outtakeDesired = outtakeButton.isDown();
		outtakeWheelsDesired = outtakeWheelsButton.isDown();

	}
	/** Reads all controller buttons **/
	public void readAllButtons() {
		//Driver
		slowDriveTier1Button.readValue();
		slowDriveTier2Button.readValue();
		driveBackButton.readValue();
		driveBackOtherButton.readValue();
		//Operator 
		
		armSwitchButton.readValue();
		armScaleButton.readValue();
		armFeedButton.readValue();
		armManualButton.readValue();
		
		intakeButton.readValue();
		outtakeButton.readValue();
		intakeButton.readValue();
		outtakeButton.readValue();
		outtakeWheelsButton.readValue();

	}

	/** Gets joystick value given joystick  and axe 
	 * 
	 * @param j A Joystick
	 * @param a An Axis
	 * 
	 * **/
	public double getJoystickValue(Joysticks j, Axes a) {
		switch (j) {
		case kDriverJoy:
			if (a == Axes.kLX) {
				return driverLeftJoyX;
			} else if (a == Axes.kLY) {
				return driverLeftJoyY;
			} else if (a == Axes.kRX) {
				return driverRightJoyX;
			} else if (a == Axes.kRY) {
				return driverRightJoyY;
			}
			break;
		case kOperatorJoy:
			if (a == Axes.kLX) {
				return operatorLeftJoyX;
			} else if (a == Axes.kLY) {
				return operatorLeftJoyY;
			} else if (a == Axes.kRX) {
				return operatorRightJoyX;
			} else if (a == Axes.kRY) {
				return operatorRightJoyY;
			}
			break;
		default:
			return 0.0;
		}
		return 0.0;
	}
	
	/*
	 * Commented in RemoteControl.java
	 * (non-Javadoc)
	 * 
	 */
	
	
	@Override
	public boolean getSlowDriveTier1Desired() {
		return slowDriveTier1Desired;
	}
	@Override
	public boolean getSlowDriveTier2Desired() {
		return slowDriveTier2Desired;
	}
	@Override
	public boolean getDriveBackDesired() {
		return driveBackDesired;
	}
	@Override
	public boolean getDriveBackOtherDesired() {
		return driveBackOtherDesired;
	}
	

	@Override
	public boolean toggleManualArmDesired() {
		SmartDashboard.putString("OPERATOR", "TOGGLE ARM");
		return toggleArmManualDesired;
	}

	@Override
	public boolean getSwitchArmDesired() {
		SmartDashboard.putString("OPERATOR", "SWITCH_POSITION");

		return armSwitchDesired;
	}

	@Override
	public boolean getScaleArmDesired() {
		SmartDashboard.putString("OPERATOR", "SCALE_POSITION");
		return armScaleDesired;
	}

	@Override
	public boolean getFeedArmDesired() {
		SmartDashboard.putString("OPERATOR", "FEED_POSITION");

		return armFeedDesired;
	}

	

	@Override
	public boolean getIntakeDesired() {
		SmartDashboard.putString("OPERATOR", "INTAKE");

		return intakeDesired;
	}

	@Override
	public boolean getOuttakeDesired() {
		SmartDashboard.putString("OPERATOR", "OUTTAKE");

		return outtakeDesired;
	}
	
	
	@Override
	public boolean outtakeWheels() {
		return outtakeWheelsDesired;
	}



}
