package org.usfirst.frc.team3997.robot.hardware;


public abstract class RemoteControl {
	/*** Whose controllers (driver or operator)
	 * 
	 * @author peter
	 *
	 */
	public enum Joysticks {
		kDriverJoy, kOperatorJoy
	};
	/*** What joystick axes (left x or right y) ***/
	public enum Axes {
		kLX, kLY, kRX, kRY
	};
	/*** Update Values ***/
	public abstract void readControls();
	/*** Update button values ***/
	public abstract void readAllButtons();
	/*** Get value of joystick axis ***/
	public abstract double getJoystickValue(Joysticks j, Axes a);

	/***If we want to drive in arcade (deprecated ***/
	public abstract boolean getArcadeDriveDesired();
	/*** Brake 1 (Slow down a little) ***/
	public abstract boolean getSlowDriveTier1Desired();
	/*** Brake 2 (Slow down a lot) ***/
	public abstract boolean getSlowDriveTier2Desired();
	
	/*** Have no idea whatsoever ***/
	public abstract boolean getDriveBackDesired();
	/**** Also no idea whatsoever ***/
	public abstract boolean getDriveBackOtherDesired();
	
	/*** ARM Manual Override ***/
	public abstract boolean toggleManualArmDesired();
	/*** Go to Switch Position (PID)***/
	public abstract boolean getSwitchArmDesired();
	/*** Go to Scale Position (PID)***/
	public abstract boolean getScaleArmDesired();
	/*** Go to Vault Position (PID)***/
	public abstract boolean getFeedArmDesired();
	
	/*** Intake block ***/
	public abstract boolean getIntakeDesired();
	/*** Outtake block ***/
	public abstract boolean getOuttakeDesired();
	/*** bring wrist down ***/
	public abstract boolean relaxWristDesired();




	
}
