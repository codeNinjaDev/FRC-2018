package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDOutput;
/*** How PID output controls arm ***/
public class ArmPIDOutput implements PIDOutput{
	
	RobotModel robot;
	
	public ArmPIDOutput(RobotModel robot) {
		this.robot = robot;
	}

	@Override
	public void pidWrite(double output) {
		//Moves arm up and down based on output
		robot.moveArm(output);
	}

}