package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotHardware;

import edu.wpi.first.wpilibj.PIDOutput;
/*** How PID output controls arm ***/
public class ArmPIDOutput implements PIDOutput{
	
	RobotHardware robot;
	
	public ArmPIDOutput(RobotHardware robot) {
		this.robot = robot;
	}

	@Override
	public void pidWrite(double output) {
		//Moves arm up and down based on output
		robot.moveArm(output);
	}

}