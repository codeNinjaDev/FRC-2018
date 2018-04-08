package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.MPU9250Gyro;
import org.usfirst.frc.team3997.robot.hardware.Ports;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class GyroPIDSource implements PIDSource {
	MPU9250Gyro gyro;
	public GyroPIDSource() {
		gyro = new MPU9250Gyro(Ports.gyroPort);
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
