package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotHardware;

import edu.wpi.first.wpilibj.PIDOutput;
/*** Sets both wheel sides as a PID output***/
public class WheelsPIDOutput implements PIDOutput {

	private RobotHardware.Wheels wheels;
	private RobotHardware robot;
	private double loopOutput;
	public WheelsPIDOutput(RobotHardware.Wheels wheels, RobotHardware robot) {
		this.wheels = wheels;
		this.robot = robot;
	}
	@Override
	public void pidWrite(double output) {
		loopOutput = output;
		robot.setWheelSpeed(wheels, output);
	}
	
	double getPIDLoopOutput() {
		return loopOutput;
	}


}
