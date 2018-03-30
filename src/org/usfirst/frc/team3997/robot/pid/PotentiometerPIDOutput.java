package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDOutput;

public class PotentiometerPIDOutput implements PIDOutput{
	
	RobotModel robot;
	
	public PotentiometerPIDOutput(RobotModel robot) {
		this.robot = robot;
	}

	@Override
	public void pidWrite(double output) {
		robot.moveArm(output);
	}

}