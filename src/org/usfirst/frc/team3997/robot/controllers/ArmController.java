package org.usfirst.frc.team3997.robot.controllers;

import java.util.ResourceBundle.Control;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl.Joysticks;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;
import org.usfirst.frc.team3997.robot.pid.PotentiometerPIDOutput;
import org.usfirst.frc.team3997.robot.pid.PotentiometerPIDSource;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/***
 * Controls arm and intake behavior
 * 
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 ***/
public class ArmController {
	/*** Instance variable with Hardware class for arm and intake ***/
	private RobotModel robot;
	/*** Instance variable for communicating with game pads ***/
	private RemoteControl humanControl;
	/*** PID controller for arm; purpose is to follow setpoints ***/
	private PIDController armPIDController;
	/*** Handles PID output from absolute arm encoder ***/
	private PIDOutput armPIDOutput;
	/*** Sets up arm encoder as a PIDSource ***/
	private PIDSource armPIDSource;

	/*** Current state of robot; e.g init or teleop ***/
	private ArmState m_stateVal;
	/*** Next state of robot; e.g init or teleop ***/
	private ArmState nextState;
	/*** Boolean for override arm control ***/
	private boolean toggleArmManual;
	/*** Boolean for collapsing intake ***/
	private boolean toggleCollapse;

	/*** Enum that lets us shift our status from init to teleop and vice versa ***/
	public enum ArmState {
		kInitialize, kTeleop
	};

	/***
	 * <h1>Constructor for ArmController</h1>
	 * <h2>Actions:</h2>
	 * <ul>
	 * <li>Initializes instance variables</li>
	 * <li>Sets toggle booleans to false</li>
	 * <li>Initializes PID Source and takes in arm encoder as a parameter</li>
	 * <li>Initializes PID output and takes in robot</li>
	 * <li>Initializes PID Controllers and takes in ARM_P, ARM_I, ARM_D, ARM_F,
	 * ARM_PID_SOURCE, ARM_PID_OUTPUT</li>
	 * <li>Sets ArmState to kInitialize</li>
	 * </ul>
	 * 
	 * @param RobotModel
	 *            robot
	 * @param RemoteControl
	 *            humanControl
	 * 
	 * 
	 * 
	 * 
	 **/
	public ArmController(RobotModel robot, RemoteControl humanControl) {
		this.robot = robot;
		this.humanControl = humanControl;

		toggleArmManual = false;
		toggleCollapse = false;

		armPIDSource = new PotentiometerPIDSource(robot.pot);
		armPIDOutput = new PotentiometerPIDOutput(robot);
		armPIDController = new PIDController(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f, armPIDSource,
				armPIDOutput);

		armPIDController.setOutputRange(-1, 1);
		armPIDController.setAbsoluteTolerance(0.25);
		armPIDController.setSetpoint(Params.ARM_REST_SETPOINT);
		armPIDController.disable();

		m_stateVal = ArmState.kInitialize;
		nextState = ArmState.kInitialize;
	}

	/***
	 * <h2>Sets arm state to kInitialize</h2>
	 ***/
	public void reset() {
		m_stateVal = ArmState.kInitialize;
	}

	/***
	 * <h2>Runs in teleop periodic
	 ***/
	public void update() {
		//Switches based off of current state
		switch (m_stateVal) {
		//If initializing
		case kInitialize:

			// Intitalize Variables and PId
			armPIDController.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
			armPIDController.setOutputRange(-1, 1);
			armPIDController.setSetpoint(Params.ARM_REST_SETPOINT);
			// Toggle variables to false
			toggleArmManual = false;
			toggleCollapse = false;
			// Sets next state to teleop
			nextState = ArmState.kTeleop;
			break;
		case kTeleop:
			// *** TOGGLE ZONE ***//

			// If armManualDesired, Toggle the arm override
			if (humanControl.toggleManualArmDesired())
				toggleArmManual = !toggleArmManual;
			// If collapseIntakeDesired, Toggle the collapse intake
			if (humanControl.toggleCollapseIntake())
				toggleCollapse = !toggleCollapse;

			// Arm Behavior
			if (toggleArmManual) {
				//Disable PID if in manual
				if (armPIDController.isEnabled()) {
					armPIDController.disable();
				}
				// Move arm based off of the Right Y of Operator Joystick
				robot.moveArm(humanControl.getJoystickValue(Joysticks.kOperatorJoy, RemoteControl.Axes.kRY));
				//Intake normally
				intakeFunctions();

			} else {
				//Disable PID
				if (armPIDController.isEnabled()) {
					armPIDController.disable();
				}
				//Intake normally
				intakeFunctions();
				//Go to set point position
				if (humanControl.getClimbArmDesired()) {
					goToClimbPosition();
				} else if (humanControl.getScaleArmDesired()) {
					goToScalePosition();
				} else if (humanControl.getSwitchArmDesired()) {
					goToSwitchPosition();
				} else if (humanControl.getFeedArmDesired()) {
					goToFeedPosition();
				}
			}
			//Set next state to teleop
			nextState = ArmState.kTeleop;
			break;
		}
		//Set current state to next state
		m_stateVal = nextState;
	}

	/*** <h2> Moves arm to scoring position for the switch **/
	public void goToSwitchPosition() {
		//Sets PID, outputrange, setpoint, and enables
		armPIDController.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_SWITCH_SETPOINT);
		armPIDController.enable();
	}
	/*** <h2> Moves arm to scoring position for the scale **/
	public void goToScalePosition() {
		//Sets PID, outputrange, setpoint, and enables
		armPIDController.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_SCALE_SETPOINT);
		armPIDController.enable();
	}
	/*** <h2> Moves arm to scoring position for the climb **/
	public void goToClimbPosition() {
		//Sets PID, outputrange, setpoint, and enables
		armPIDController.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_CLIMB_SETPOINT);
		armPIDController.enable();
	}
	/*** <h2> Moves arm to scoring position for the vault **/
	public void goToFeedPosition() {
		//Sets PID, outputrange, setpoint, and enables
		armPIDController.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_FEED_SETPOINT);
		armPIDController.enable();
	}
	/*** <h2> Stops all arm movement **/
	public void stop() {
		//Disables PID
		armPIDController.disable();
		//Stops arm
		robot.moveArm(0);
		//Stops intake
		robot.stopIntake();
	}
	/*** Intakes Power Cube ***/
	public void intakePowerCube() {
		//Snaps wrist down
		robot.relaxWrist();
		//Intake wheels
		robot.intakeBlock();
		//Closes intakes
		robot.closeIntake();
	}

	public void outtakePowerCube() {
		//Snaps wrist up
		robot.flexWrist();
		//Outtake wheels
		robot.outtakeBlock();
		//Open intake
		robot.openIntake();
	}

	public void intakeFunctions() {
		// If intake button pressed run intake wheels
		if (humanControl.getIntakeDesired()) {
			// TODO Add limit switch logic
			intakePowerCube();

			// If outtake button pressed openIntake and run outtake wheels
		} else if (humanControl.getOuttakeDesired()) {
			// TODO Add limit switch logic
			outtakePowerCube();

		} else {
			robot.stopIntake();
		}
		// If toggle collapsed is desired then it shifts the pneumatics to the next
		// position
		if (toggleCollapse) {
			robot.closeIntake();
		} else {
			robot.openIntake();
		}
	}

}