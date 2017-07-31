package org.usfirst.frc.team3997.robot.pid;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDOutput;
// Mainly for gear pot 
public class AnalogPIDOutput implements PIDOutput{
	
	RobotModel robot;
	
	public AnalogPIDOutput(RobotModel robot) {
		this.robot = robot;
	}

	@Override
	public void pidWrite(double output) {
		robot.setGearTilterSpeed(output);
	}

}
