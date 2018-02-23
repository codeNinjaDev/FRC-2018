package org.usfirst.frc.team3997.robot.feed;

import org.usfirst.frc.team3997.robot.Params;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardInput {

	Preferences preferences;

	public DashboardInput() {
		preferences = Preferences.getInstance();

		DashboardVariables.firstAutoDistance = preferences.getDouble("First Auto Distance", 0);

		DashboardVariables.nextAutoAngle = preferences.getDouble("Next Auto Angle", 0);

		DashboardVariables.lastAutoDistance = preferences.getDouble("Second Auto Distance", 0);

		Params.drive_p = preferences.getDouble("Drive P Value", 0);
		Params.drive_i = preferences.getDouble("Drive I Value", 0);
		Params.drive_d = preferences.getDouble("Drive D Value", 0);

		Params.MAX_SPEED = preferences.getDouble("Max Speed", 1);
		System.out.println("PREFS: " + preferences.getDouble("Max Speed", 1));
	}

	public void updateInput() {
		preferences = Preferences.getInstance();
		DashboardVariables.firstAutoDistance = preferences.getDouble("First Auto Distance", 0);

		DashboardVariables.nextAutoAngle = preferences.getDouble("Next Auto Angle", 0);

		DashboardVariables.lastAutoDistance = preferences.getDouble("Second Auto Distance", 0);

		Params.drive_p = preferences.getDouble("Drive P Value", 0);
		Params.drive_i = preferences.getDouble("Drive I Value", 0);
		Params.drive_d = preferences.getDouble("Drive D Value", 0);

		Params.MAX_SPEED = preferences.getDouble("Max Speed", 1);

	}

}
