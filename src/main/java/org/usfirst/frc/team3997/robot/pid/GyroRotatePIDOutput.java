package org.usfirst.frc.team3997.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/*** Rotates drive based on output from PID Controller ***/
public class GyroRotatePIDOutput implements PIDOutput{
	private DifferentialDrive drive;
	
	public GyroRotatePIDOutput(DifferentialDrive drive) {
		this.drive = drive;
	}
	@Override
	public void pidWrite(double angle) {
		drive.arcadeDrive(0.0, angle, false);

	}

}
