package org.usfirst.frc.team3997.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
/*** Waits for n seconds ***/
public class WaitAction extends Command {
	double start_time, goal_time;
	/**
	 * Delays auto
	 * @param seconds Num seconds of delay
	 */
	public WaitAction(double seconds) {
		goal_time = seconds;
		
	}
	
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() >= start_time + goal_time);
	}


	public void execute() {
		
	}

	public void end() {
		
	}

	public void initialize() {
		start_time = Timer.getFPGATimestamp();
	}
	
	protected void interrupt() {
		end();
	}
}
