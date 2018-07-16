package org.usfirst.frc.team3997.robot.feed;

import org.usfirst.frc.team3997.robot.Params;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Allows input from Robot Preferences in SmartDashboard ***/
public class DashboardInput {
	/*** Preferences object to get input ***/
	Preferences preferences;
	//TODO Check if Dashboard Variables or Params works better
	/*** Get Instance of preferences and Update input ***/
	public DashboardInput() {
		//Get overall Input from preferences
		preferences = Preferences.getInstance();

		//Set ARM Setpoints equal to preferences ARM Setpoints
		Params.ARM_CLIMB_SETPOINT = preferences.getDouble("CLIMB_ARM_ANGLE", 150);
		Params.ARM_SCALE_SETPOINT = preferences.getDouble("SCALE_ARM_ANGLE", 125);
		Params.ARM_SWITCH_SETPOINT = preferences.getDouble("SWITCH_ARM_ANGLE", 48);
		Params.ARM_FEED_SETPOINT = preferences.getDouble("FEED_ARM_ANGLE", 10);
		//Set Max Speed to preferences Max Speed
		DashboardVariables.max_speed = preferences.getDouble("MAX_SPEED", 1);
		
		
		DashboardVariables.firstAutoTime = preferences.getDouble("AUTO_TIME", 0);

		Params.TIME_DELAY = preferences.getDouble("AUTO_DELAY", 0);
		Params.MAX_SPEED = preferences.getDouble("MAX_SPEED", 1);
		
		Params.track_base_width = preferences.getDouble("TRACK_BASE_WIDTH", 0);
		Params.wheel_base_width = preferences.getDouble("WHEEL_BASE_WIDTH", 0);
		Params.dt = preferences.getDouble("DELTA_TIME_MP", 0);
	}
	
	
	
	/*** Updates input from Dashboard ***/
	public void updateInput() {
		//Gets information from dashboard
		preferences = Preferences.getInstance();
		//Gets Custom Autonomous Time
		DashboardVariables.firstAutoTime = preferences.getDouble("AUTO_TIME", 0);
		

		//Gets ARM SETPOINTS and logs it
		Params.ARM_CLIMB_SETPOINT = preferences.getDouble("CLIMB_ARM_ANGLE", 150);
		Params.ARM_SCALE_SETPOINT = preferences.getDouble("SCALE_ARM_ANGLE", 125);
		Params.ARM_SWITCH_SETPOINT = preferences.getDouble("SWITCH_ARM_ANGLE", 48);
		Params.ARM_FEED_SETPOINT = preferences.getDouble("FEED_ARM_ANGLE", 10);

		//Gets max speed
		DashboardVariables.max_speed = preferences.getDouble("MAX_SPEED", 1);
		SmartDashboard.putNumber("Dash Max Speed", DashboardVariables.max_speed);

		
		
		Params.MAX_SPEED = preferences.getDouble("MAX_SPEED", 1);
		
		Params.TIME_DELAY = preferences.getDouble("AUTO_DELAY", 0);

		Params.track_base_width = preferences.getDouble("TRACK_BASE_WIDTH", 0);
		Params.wheel_base_width = preferences.getDouble("WHEEL_BASE_WIDTH", 0);
		Params.dt = preferences.getDouble("DELTA_TIME_MP", 0);

	}

}
