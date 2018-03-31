
package org.usfirst.frc.team3997.robot.hardware;

import org.usfirst.frc.team3997.robot.Params;
import edu.wpi.first.wpilibj.*;
/** Handles inputs from controller 
 * @category hardware
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 * 
 * **/
public class ControlBoard extends RemoteControl {
	/** Driver Buttons **/
	public ButtonReader arcadeDriveButton, tankDriveButton, driveBackButton, driveBackOtherButton;
	//Operator Buttons
/** Operator Buttons **/
	public ButtonReader armManualButton, armSwitchButton, armScaleButton, armFeedButton, armClimbButton, intakeButton, outtakeButton, openIntakeButton, closeIntakeButton;

/** Driver Triggers **/
	public TriggerReader slowDriveTier1Button, slowDriveTier2Button;
	/** Operator Triggers **/
	public TriggerReader flexWristButton, relaxWristButton;
	
	private boolean openIntakeDesired, closeIntakeDesired, flexWristDesired, relaxWristDesired, arcadeDriveDesired, slowDriveTier1Desired, slowDriveTier2Desired,
			driveBackDesired, driveBackOtherDesired, toggleArmManualDesired, armSwitchDesired, armScaleDesired, armFeedDesired, armClimbDesired, intakeDesired, outtakeDesired, armShifterDesired;


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
			arcadeDriveButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_START_BUTTON);
			tankDriveButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_BACK_BUTTON);
			driveBackButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_LEFT_BUMPER);
			driveBackOtherButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_RIGHT_BUMPER);
			
			slowDriveTier1Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS);
			slowDriveTier2Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_LEFT_TRIGGER_AXIS);
			
			//Operator Controls
			armScaleButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_YELLOW_BUTTON);
			armSwitchButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_GREEN_BUTTON);
			armClimbButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_BUMPER);
			armFeedButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_LEFT_BUMPER);
			armManualButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_BACK_BUTTON);
			
			flexWristButton = new TriggerReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS);
			relaxWristButton = new TriggerReader(operatorJoy, XInput.XINPUT_WIN_LEFT_TRIGGER_AXIS);
			intakeButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_BLUE_BUTTON);
			outtakeButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_RED_BUTTON);
			openIntakeButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_YELLOW_BUTTON);
			closeIntakeButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_GREEN_BUTTON);
			
		}

		driverLeftJoyX = 0;
		driverLeftJoyY = 0;
		driverRightJoyX = 0;
		driverRightJoyY = 0;

		// Driver variables
		arcadeDriveDesired = Params.USE_ARCADE_DRIVE;
		
		
		slowDriveTier1Desired = false;
		slowDriveTier2Desired = false;
		driveBackDesired = false;
		driveBackOtherDesired = false;
		
		//Operator Vars
		armSwitchDesired = false;
		armScaleDesired = false;
		armFeedDesired = false;
		toggleArmManualDesired = false;
		armClimbDesired = false;
		
		intakeDesired = false;
		outtakeDesired = false;
		flexWristDesired = false;
		relaxWristDesired = false;
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
		arcadeDriveDesired = arcadeDriveButton.isDown();

		slowDriveTier1Desired = slowDriveTier1Button.isDown();
		slowDriveTier2Desired = slowDriveTier2Button.isDown();
		driveBackDesired = driveBackButton.isDown();
		driveBackOtherDesired = driveBackOtherButton.isDown();
		
		//Operator Vars
		armSwitchDesired = armSwitchButton.isDown();
		armScaleDesired = armScaleButton.isDown();
		armClimbDesired = armClimbButton.isDown();
		armFeedDesired = armFeedButton.isDown();
		toggleArmManualDesired = armManualButton.isDown();
		flexWristDesired = flexWristButton.isDown();
		relaxWristDesired = relaxWristButton.isDown();
		intakeDesired = intakeButton.isDown();
		outtakeDesired = outtakeButton.isDown();
		openIntakeDesired = openIntakeButton.isDown();
		closeIntakeDesired = closeIntakeButton.isDown();
	}
	/** Reads all controller buttons **/
	public void readAllButtons() {
		//Driver
		arcadeDriveButton.readValue();
		tankDriveButton.readValue();
		slowDriveTier1Button.readValue();
		slowDriveTier2Button.readValue();
		driveBackButton.readValue();
		driveBackOtherButton.readValue();
		//Operator 
		
		armClimbButton.readValue();
		armSwitchButton.readValue();
		armScaleButton.readValue();
		armFeedButton.readValue();
		armManualButton.readValue();
		
		intakeButton.readValue();
		outtakeButton.readValue();
		intakeButton.readValue();
		outtakeButton.readValue();
		flexWristButton.readValue();
		relaxWristButton.readValue();
		openIntakeButton.readValue();
		closeIntakeButton.readValue();
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
	

	/** Checks if driver wants arcade drive **/
	public boolean getArcadeDriveDesired() {
		return arcadeDriveDesired;
	}
	/** Checks if driver wants first brake **/
	public boolean getSlowDriveTier1Desired() {
		return slowDriveTier1Desired;
	}
	/** Checks if driver wants second brake **/
	public boolean getSlowDriveTier2Desired() {
		return slowDriveTier2Desired;
	}
	/** Checks if driver wants inverse drive **/
	public boolean getDriveBackDesired() {
		return driveBackDesired;
	}
	/** I have no clue **/
	public boolean getDriveBackOtherDesired() {
		return driveBackOtherDesired;
	}
	
	

	@Override
	public boolean toggleManualArmDesired() {
		return toggleArmManualDesired;
	}

	@Override
	public boolean getSwitchArmDesired() {
		return armSwitchDesired;
	}

	@Override
	public boolean getScaleArmDesired() {
		return armScaleDesired;
	}

	@Override
	public boolean getFeedArmDesired() {
		return armFeedDesired;
	}

	@Override
	public boolean getClimbArmDesired() {
		return armClimbDesired;
	}

	@Override
	public boolean getIntakeDesired() {
		return intakeDesired;
	}

	@Override
	public boolean getOuttakeDesired() {
		return outtakeDesired;
	}
	@Override
	public boolean flexWristDesired() {
		return flexWristDesired;
	}
	@Override
	public boolean relaxWristDesired() {
		return relaxWristDesired;
	}

	@Override 
	public boolean openIntakeDesired() {
		return openIntakeDesired;
	}
	
	@Override 
	public boolean closeIntakeDesired() {
		return closeIntakeDesired;
	}

}
