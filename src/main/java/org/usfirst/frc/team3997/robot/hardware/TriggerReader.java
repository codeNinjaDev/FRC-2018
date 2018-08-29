package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Reads Trigger Input *****/
public class TriggerReader {
	/*** Gamepad that contains trigger. Either driver or operator */
	private Joystick joystick;
	/*** Mapping for trigger on gamepad */
	private int triggerAxis;
	/*** Previous State */
	private boolean lastState;
	/*** Current State */
	private boolean currState;
	/*** Description of what the trigger controls */
	private String triggerPurpose;
	/*** The threshold at which we consider the trigger pressed down */
	private double PRESSED_THRESHOLD = 0.8;

	/** Initializes controller variables 
	 * @param joystick The controller 
	 * @param triggerAxis The mapping of the trigger
	 * @param triggerPurpose Describes function of the trigger **/
	public TriggerReader(Joystick joystick, int triggerAxis, String triggerPurpose) {
		this.joystick = joystick;
		this.triggerAxis = triggerAxis;
		this.triggerPurpose = triggerPurpose + "_TRIGGER";
		if(joystick.getRawAxis(triggerAxis) > PRESSED_THRESHOLD) {
			currState = true;
		} else {
			currState = false;
		}
		lastState = currState;
	}
	/*** Updates the value of the trigger */
	public void readValue() {
		lastState = currState;
		SmartDashboard.putNumber(triggerPurpose + "_AXIS", joystick.getRawAxis(triggerAxis));
		if(joystick.getRawAxis(triggerAxis) > PRESSED_THRESHOLD) {
			currState = true;
		} else {
			currState = false;
		}
	}
	
	/*** Checks if the trigger was just pressed */
	public boolean wasJustPressed() {
		return (lastState == false && currState == true);
	}
	
	/*** Checks if the trigger was just released */
	public boolean wasJustReleased() {
		return (lastState == true && currState == false);
	}
	
	/*** Checks if the trigger state has switched */
	public boolean stateJustChanged() {
		return (lastState != currState);
	}
	
	/*** Checks if the trigger is currently being pressed */
	public boolean isDown() {
		SmartDashboard.putBoolean(triggerPurpose, currState);
		return currState;
	}

}
