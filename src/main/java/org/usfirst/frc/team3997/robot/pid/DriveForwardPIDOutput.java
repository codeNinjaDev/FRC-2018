package org.usfirst.frc.team3997.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/*** Drives Straight based on PIDOutput ***/
public class DriveForwardPIDOutput implements PIDOutput {

	private DifferentialDrive drive;
	public DriveForwardPIDOutput(DifferentialDrive drive) {
		this.drive = drive;
	}
	@Override
	public void pidWrite(double output) {
		drive.arcadeDrive(output, 0, false);
	}
	
}
