/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import jaci.pathfinder.Trajectory;

/**
 * @author peter
 *
 */
public class CenterAutoRoutine extends AutoRoutine{
	private MasterController controllers;
	public CenterAutoRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
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

		}
	}

	@Override
	protected void routine() {

	}


}
