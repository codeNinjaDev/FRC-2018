/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author peter
 *
 */
public class CenterAutoRoutine extends CommandGroup{
	private MasterController controllers;
	public CenterAutoRoutine(MasterController controllers) {
		this.controllers = controllers;
		boolean isLeftSwitch = (PlateDetector.getSwitchColor() == 'L');
		if(isLeftSwitch) {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			if(isLeftScale) {
				
			} else {
				
			}
			

		} else {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			if(isLeftScale) {
				
			} else {
				
			}

		}	}

	

}
