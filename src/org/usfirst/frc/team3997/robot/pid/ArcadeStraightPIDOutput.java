package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Uses arcade drive rather than tank to drive straight
public class ArcadeStraightPIDOutput implements PIDOutput {
	private RobotDrive drive;
	private RobotModel robot;
	private double loopOutput;
	private double kPEncoder; //0.625
	public ArcadeStraightPIDOutput(RobotDrive drive, RobotModel robot) {
		this.drive = drive;
		this.robot = robot;
		kPEncoder = 0.625;
	}
	@Override
	public void pidWrite(double output) {
		loopOutput = output;
		drive.arcadeDrive(output, robot.getEncoderError() * kPEncoder, false);
	    SmartDashboard.putNumber("ArcadeCORRECTION", robot.getEncoderError() * kPEncoder);
	    SmartDashboard.putNumber("auton_EncoderError", robot.getEncoderError());
	}
	
	double getPIDLoopOutput() {
		return loopOutput;
	}


}
