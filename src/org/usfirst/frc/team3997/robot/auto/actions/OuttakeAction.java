package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;
/***
 * Outtakes block for certain amount
 * @author peter
 *
 */
public class OuttakeAction extends Action {
	
	double speed;
	RobotModel robot;
	/***
	 * Outtakes block given time and speeed
	 * @param controllers Master Controller that gives access to all robot functionality
	 * @param timeout Time alotted for action
	 * @param speed Speed of wheels
	 */
	public OuttakeAction(MasterController controllers, double timeout, double speed) {
		goal_time = timeout;
		robot = controllers.getRobotModel();
		this.speed = speed;
		
	}
	
	/*** Checks if action exceeded timeout ***/
	public boolean isFinished() {
		return ((Timer.getFPGATimestamp() >= start_time + goal_time));
	}

	@Override
	/*** Runs outtake wheels ***/
	public void update() {
		robot.intakeWheels(-speed);

	}

	@Override
	/*** Opens intake and stops wheels***/
	public void finish() {

		//robot.outtakeBlock();
		robot.openIntake();
		robot.stopIntake();

	}

	@Override
	/*** starts timer ***/
	public void start() {
		start_time = Timer.getFPGATimestamp();
	}

}
