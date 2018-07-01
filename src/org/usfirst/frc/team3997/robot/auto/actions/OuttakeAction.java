package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
/***
 * Outtakes block for certain amount
 * @author peter
 *
 */
public class OuttakeAction extends Command {
	
	double speed, goal_time, start_time;
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
	protected boolean isFinished() {
		return ((Timer.getFPGATimestamp() >= start_time + goal_time));
	}

	/*** Runs outtake wheels ***/
	protected void execute() {
		robot.intakeWheels(-speed);

	}

	/*** Opens intake and stops wheels***/
	protected void end() {

		//robot.outtakeBlock();
		robot.openIntake();
		robot.stopIntake();

	}

	/*** starts timer ***/
	protected void initialize() {
		start_time = Timer.getFPGATimestamp();
	}

	protected void interrupt() {
		end();
	}
}
