package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Creates a toggle button on gamepad ***/
public class ToggleButtonReader extends ButtonReader {
	/*** The current state of the toggle */
	private boolean currToggleState;
	/*** Description of the function that the toggle controls */
	private String togglePurpose;
	/***
	 * Initializes a toggle button reader as a new button reader
	 * @param joy The gamepad that contains the button
	 * @param buttonNum The numerical mapping of the button
	 * @param togglePurpose Purpose of the toggle button
	 */

	public ToggleButtonReader(Joystick joy, int buttonNum, String togglePurpose) {
		super(joy, buttonNum, togglePurpose);
		this.togglePurpose = togglePurpose + "_TOGGLE";
		currToggleState = false;
	}
	/**
	 * Returns the current boolean state of the toggle button
	 */
	public boolean getState() {
		if (wasJustReleased()) {
			currToggleState = !currToggleState;
		}
		SmartDashboard.putBoolean(togglePurpose, currToggleState);
		return (currToggleState);
	}
}