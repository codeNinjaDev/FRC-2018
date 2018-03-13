package org.usfirst.frc.team3997.robot.controllers;

import java.util.ResourceBundle.Control;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl.Joysticks;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;
import org.usfirst.frc.team3997.robot.pid.AbsoluteEncoderPIDOutput;
import org.usfirst.frc.team3997.robot.pid.AbsoluteEncoderPIDSource;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;


public class ArmController {
	private RobotModel robot;
	private RemoteControl humanControl;
	private PIDController armPIDController;
	private PIDOutput armPIDOutput;
	private PIDSource armPIDSource;

	private ArmState m_stateVal;
	private ArmState nextState;
	private boolean toggleArmManual;
	private boolean toggleCollapse;
	private boolean toggleArmShifter;

	public enum ArmState {
		kInitialize, kTeleop
	};

	public ArmController(RobotModel robot, RemoteControl humanControl) {
		this.robot = robot;
		this.humanControl = humanControl;
		
		toggleArmManual = false;
		toggleArmShifter = false;
		toggleCollapse = false;
		
		armPIDSource = new AbsoluteEncoderPIDSource(robot.armEncoder);
		armPIDOutput = new AbsoluteEncoderPIDOutput(robot);
		armPIDController = new PIDController(0, 0, 0, 0, armPIDSource, armPIDOutput);
		
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setAbsoluteTolerance(0.25);
		armPIDController.setSetpoint(Params.ARM_REST_SETPOINT);
		armPIDController.disable();
		
		
		m_stateVal = ArmState.kInitialize;
		nextState = ArmState.kInitialize;
	}

	public void reset() {
		m_stateVal = ArmState.kInitialize;
	}

	// ???
	public void update() {
		switch(m_stateVal) {
		case kInitialize:
			
			//Intitalize Variables and PId
			armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
			armPIDController.setOutputRange(-1, 1);
			armPIDController.setSetpoint(Params.ARM_REST_SETPOINT);
			toggleArmManual = false;
			toggleCollapse = false;
			toggleArmShifter = false;
			nextState = ArmState.kTeleop;
			break;
		case kTeleop:
			//*** TOGGLE ZONE ***//
			
			//Toggle the arm override
			if(humanControl.toggleManualArmDesired())
				toggleArmManual = !toggleArmManual;
			if(humanControl.toggleCollapseIntake()) 
				toggleCollapse = !toggleCollapse;
			if(humanControl.toggleArmShifter())
				toggleArmShifter = !toggleArmShifter;
			
			if(toggleArmShifter)
				robot.setArmLowGear();
			else
				robot.setArmHighGear();
			
			//Arm Behavior
			if (toggleArmManual) {
				//Move arm based off of the Right Y of Operator Joystick
				if (armPIDController.isEnabled()) {
					armPIDController.disable();
				}
				robot.moveArm(humanControl.getJoystickValue(Joysticks.kOperatorJoy, RemoteControl.Axes.kRY));
				intakeFunctions();
				
			} else {
				if (armPIDController.isEnabled()) {
					armPIDController.disable();
				}
				
				intakeFunctions();
				if(humanControl.getClimbArmDesired()) {
					goToClimbPosition();
				} else if(humanControl.getScaleArmDesired()) {
					goToScalePosition();
				} else if(humanControl.getSwitchArmDesired()) {
					goToSwitchPosition();
				} else if(humanControl.getFeedArmDesired()) {
					goToFeedPosition();
				}
			}
			
			
			nextState = ArmState.kTeleop;
			break;
		}
		m_stateVal = nextState;
	}
	
	public void goToSwitchPosition() {
		armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_SWITCH_SETPOINT);
		armPIDController.enable();
	}
	
	public void goToScalePosition() {
		armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_SCALE_SETPOINT);
		armPIDController.enable();
	}
	
	public void goToClimbPosition() {
		armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_CLIMB_SETPOINT);
		armPIDController.enable();
	}
	
	public void goToFeedPosition() {
		armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_FEED_SETPOINT);
		armPIDController.enable();
	}
	
	
	public void stop() {
		armPIDController.disable();
		robot.moveArm(0);
		robot.stopIntake();
	}
	
	public void intakeFunctions() {
		//If intake button pressed run intake wheels
		if(humanControl.getIntakeDesired()) {
			robot.intakeBlock();
			robot.closeIntake();
	    //If outtake button pressed openIntake and run outtake wheels
		} else if(humanControl.getOuttakeDesired()) {
			robot.outtakeBlock();
			robot.openIntake();
			
		} else {
			robot.stopIntake();
		}
		// If toggle collapsed is desired then it shifts the pneumatics to the next position
		if(toggleCollapse) {
			robot.closeIntake();
		} else {
			robot.openIntake();
		}
	}

}