package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.MPU9250Gyro;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
/*** Sets MPU9250 gyro as a PID Source ***/
public class GyroPIDSource implements PIDSource {
	MPU9250Gyro gyro;
	public GyroPIDSource(MPU9250Gyro gyro) {
		this.gyro = gyro;
	}
	@Override
	public PIDSourceType getPIDSourceType() {
		return gyro.getPIDSourceType();
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return gyro.pidGet();
	}

	@Override
	public void setPIDSourceType(PIDSourceType type) {
		gyro.setPIDSourceType(type);
	}

}