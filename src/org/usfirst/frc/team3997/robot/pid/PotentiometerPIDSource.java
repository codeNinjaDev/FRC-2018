package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.TenTurnPotentiometer;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;

public class PotentiometerPIDSource implements PIDSource {
	TenTurnPotentiometer pot;
	double currentAngle;
	double pastAngle;
	
	double currentTime;
	double pastTime;
	Timer armTimer;

	public PotentiometerPIDSource(TenTurnPotentiometer pot) {
		
		this.pot = pot;
		pastAngle = pot.getAngle();
		currentAngle = pot.getAngle();
		armTimer = new Timer();
		armTimer.start();
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		pot.setPIDSourceType(pidSource);
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return pot.getPIDSourceType();
	}

	@Override
	public double pidGet() {
		switch (getPIDSourceType()) {
		case kDisplacement:
			return pot.getAngle();
		case kRate:
			currentTime = armTimer.get();
			currentAngle = pot.getAngle();
			double deltaAngle = currentAngle - pastAngle;
			double deltaTime = currentTime - pastTime;
			
			double anglePerSecond =  deltaAngle/deltaTime;
			
			pastTime = currentTime;
			pastAngle = currentAngle;
			return anglePerSecond;
		default:
			return 0;
		}
	}

}