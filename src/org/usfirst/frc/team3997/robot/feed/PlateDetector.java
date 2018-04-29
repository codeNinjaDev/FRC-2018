/**
 * 
 */
package org.usfirst.frc.team3997.robot.feed;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * @author peter
 *
 */
public class PlateDetector {
	/**
	 * 
	 */
	
	/*** Gets our alliance's switch color ***/
	public static char getSwitchColor() {
		//Tries to get color
		try {
			String gameData = DriverStation.getInstance().getGameSpecificMessage();
			//Returns first char (switch)
			return gameData.charAt(0);
		} catch (Exception e) {
			System.out.println("Cannot Find GameData");
		}
		return ' ';

	}
	/*** Gets scale color ***/
	public static char getScaleColor() {
		//Tries to get color
		try {
			String gameData = DriverStation.getInstance().getGameSpecificMessage();
			//Returns second char (scale)
			return gameData.charAt(1);
		} catch (Exception e) {
			System.out.println("Cannot Find GameData");
		}
		return ' ';
	}
	
	
	

}
