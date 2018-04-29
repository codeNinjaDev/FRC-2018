package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
/*** PID Source for Driving with Encoders ***/
public class DriveEncodersPIDSource implements PIDSource {
	private RobotModel robot;
	/*** Averages encoder values and returns them to PID Controller ***/
	public DriveEncodersPIDSource(RobotModel robot) {
		this.robot = robot;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return robot.leftDriveEncoder.getPIDSourceType();
		//TODO Add right encoder somehow
	}
	@Override
	public double pidGet() {
		switch (getPIDSourceType()) {
	    case kDisplacement:
	      return getAverageDistance();
	    case kRate:
	      return getAverageRate();
	    default:
	      return 0.0;
	  }
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kRate);
		robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kRate);		
	}
	
	public double getAverageDistance() {
		return ((robot.leftDriveEncoder.getDistance()) + (robot.rightDriveEncoder.getDistance())) / 2.0;	
	}
	public double getAverageRate() {
		return ((robot.leftDriveEncoder.getRate()) + (robot.rightDriveEncoder.getRate())) / 2.0;	
	}

}
