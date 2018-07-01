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


		//Set ARM PID equal to preferences ARM PID
		Params.arm_p = preferences.getDouble("ARM_P_VALUE", 0);
		Params.arm_i = preferences.getDouble("ARM_I_VALUE", 0);
		Params.arm_d = preferences.getDouble("ARM_D_VALUE", 0);
		Params.arm_f = preferences.getDouble("ARM_F_VALUE", 0);

		//Set ARM Setpoints equal to preferences ARM Setpoints
		Params.ARM_CLIMB_SETPOINT = preferences.getDouble("CLIMB_ARM_ANGLE", 150);
		Params.ARM_SCALE_SETPOINT = preferences.getDouble("SCALE_ARM_ANGLE", 125);
		Params.ARM_SWITCH_SETPOINT = preferences.getDouble("SWITCH_ARM_ANGLE", 48);
		Params.ARM_FEED_SETPOINT = preferences.getDouble("FEED_ARM_ANGLE", 10);
		//Set Max Speed to preferences Max Speed
		DashboardVariables.max_speed = preferences.getDouble("MAX_SPEED", 1);
		Params.drive_p = preferences.getDouble("DRIVE_P_VALUE", 0);
		Params.drive_i = preferences.getDouble("DRIVE_I_VALUE", 0);
		Params.drive_d = preferences.getDouble("DRIVE_D_VALUE", 0);

		DashboardVariables.firstAutoTime = preferences.getDouble("AUTO_TIME", 0);
		SmartDashboard.putNumber("ARM P", Params.arm_p);
		Params.TIME_DELAY = preferences.getDouble("AUTO_DELAY", 0);
		Params.MAX_SPEED = preferences.getDouble("MAX_SPEED", 1);
		System.out.println("PREFS: " + preferences.getDouble("Max Speed", 1));
	}
	/*** Updates input from Dashboard ***/
	public void updateInput() {
		//Gets information from dashboard
		preferences = Preferences.getInstance();
		//Gets Custom Autonomous Time
		DashboardVariables.firstAutoTime = preferences.getDouble("AUTO_TIME", 0);
		
		//Gets ARM PID from dashboard, and logs it
		Params.arm_p = preferences.getDouble("ARM_P_VALUE", 0);
		SmartDashboard.putNumber("Arm P", Params.arm_p);
		Params.arm_i = preferences.getDouble("ARM_I_VALUE", 0);
		SmartDashboard.putNumber("Arm I", Params.arm_i);
		Params.arm_d = preferences.getDouble("ARM_D_VALUE", 0);
		SmartDashboard.putNumber("Arm D", Params.arm_d);
		Params.arm_f = preferences.getDouble("ARM_F_VALUE", 0);
		SmartDashboard.putNumber("Arm F", Params.arm_f);

		//Gets ARM SETPOINTS and logs it
		Params.ARM_CLIMB_SETPOINT = preferences.getDouble("CLIMB_ARM_ANGLE", 150);
		SmartDashboard.putNumber("CLimb Setpoint", Params.ARM_CLIMB_SETPOINT);
		Params.ARM_SCALE_SETPOINT = preferences.getDouble("SCALE_ARM_ANGLE", 125);
		SmartDashboard.putNumber("Scale Setpoint", Params.ARM_SCALE_SETPOINT);
		Params.ARM_SWITCH_SETPOINT = preferences.getDouble("SWITCH_ARM_ANGLE", 48);
		SmartDashboard.putNumber("Switch Setpoint", Params.ARM_SWITCH_SETPOINT);
		Params.ARM_FEED_SETPOINT = preferences.getDouble("FEED_ARM_ANGLE", 10);
		SmartDashboard.putNumber("Feed Setpoint", Params.ARM_FEED_SETPOINT);

		//Gets max speed
		DashboardVariables.max_speed = preferences.getDouble("MAX_SPEED", 1);
		SmartDashboard.putNumber("Dash Max Speed", DashboardVariables.max_speed);

		//Gets Drive PID and logs it
		Params.drive_p = preferences.getDouble("DRIVE_P_VALUE", 0);
		SmartDashboard.putNumber("Drive P", Params.drive_p);
		Params.drive_i = preferences.getDouble("DRIVE_I_VALUE", 0);
		SmartDashboard.putNumber("Drive I", Params.drive_i);
		Params.drive_d = preferences.getDouble("DRIVE_D_VALUE", 0);
		SmartDashboard.putNumber("Drive D", Params.drive_d);

		Params.MAX_SPEED = preferences.getDouble("MAX_SPEED", 1);
		SmartDashboard.putNumber("Params Max Speed", Params.MAX_SPEED);
		SmartDashboard.putNumber("ARM P", Params.arm_p);
		Params.TIME_DELAY = preferences.getDouble("AUTO_DELAY", 0);


	}

}
