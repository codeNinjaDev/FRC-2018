package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.Robot;
import org.usfirst.frc.team3997.robot.hardware.RobotHardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
/***
 * Outtakes block for certain amount
 * @author peter
 *
 */
public class OuttakeAction extends Command {
	
	double speed, goal_time, start_time;
	RobotHardware robot;
	/***
	 * Outtakes block given time and speeed
	 * @param controllers Master Controller that gives access to all robot functionality
	 * @param timeout Time alotted for action
	 * @param speed Speed of wheels
	 */
	public OuttakeAction(double timeout, double speed) {
		requires(Robot.robot);
		
		goal_time = timeout;
		robot = Robot.robot;
		this.speed = speed;
		
	}
	
	/*** Checks if action exceeded timeout ***/
	public boolean isFinished() {
		return ((Timer.getFPGATimestamp() >= start_time + goal_time));
	}

	@Override
	/*** Runs outtake wheels ***/
	public void execute() {
		robot.intakeWheels(-speed);

	}

	@Override
	/*** Opens intake and stops wheels***/
	public void end() {

		//robot.outtakeBlock();
		robot.openIntake();
		robot.stopIntake();

	}
	
	public void interrupt() {
		end();
	}

	@Override
	/*** starts timer ***/
	public void initialize() {
		start_time = Timer.getFPGATimestamp();
	}

}
