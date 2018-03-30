package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.TenTurnPotentiometer;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PotentiometerPIDSource implements PIDSource {
	TenTurnPotentiometer pot;

	public PotentiometerPIDSource(TenTurnPotentiometer pot) {
		this.pot = pot;
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
			return 0.0;
		default:
			return 0;
		}
	}

}