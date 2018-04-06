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
	
	
	public static char getSwitchColor() {
		try {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		return gameData.charAt(0);
		} catch (Exception e) {
			System.out.println("Cannot Find GameData");
		}
		return ' ';

	}
	public static char getScaleColor() {
		try {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		return gameData.charAt(1);
		} catch (Exception e) {
			System.out.println("Cannot Find GameData");
		}
		return ' ';
	}
	
	
	

}
