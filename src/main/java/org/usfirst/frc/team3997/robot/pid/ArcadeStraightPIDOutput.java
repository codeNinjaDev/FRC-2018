package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotHardware;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Controls how a PID controller outputs to the robot in arcade drive ***/
public class ArcadeStraightPIDOutput implements PIDOutput {
	private DifferentialDrive drive;
	private RobotHardware robot;
	private double loopOutput;
	//P for rotation
	private double kPencoder; //0.625
	public ArcadeStraightPIDOutput(DifferentialDrive drive, RobotHardware robot) {
		this.drive = drive;
		this.robot = robot;
		kPencoder = 0.625;
	}
	@Override
	public void pidWrite(double output) {
		loopOutput = output;
		//Drives straight based on output
		drive.arcadeDrive(output, robot.getEncoderError() * kPencoder, false);
	    SmartDashboard.putNumber("ArcadeCORRECTION", robot.getEncoderError() * kPencoder);
	    SmartDashboard.putNumber("auton_EncoderError", robot.getEncoderError());
	}
	
	double getPIDLoopOutput() {
		return loopOutput;
	}


}
