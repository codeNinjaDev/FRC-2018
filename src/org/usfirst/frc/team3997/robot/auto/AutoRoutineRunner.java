package org.usfirst.frc.team3997.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoRoutineRunner {
	static Timer autoTimer = new Timer();
	private boolean autoStarted;
	private AutoRoutine new_auto_routine;
	private Thread routine_thread;
	/*** 
	 * Starts auto routine 
	 * @param new_auto_routine 
	 */
	public void setAutoRoutine(AutoRoutine new_auto_routine) {
		
		autoTimer.reset();
		autoStarted = false;
		this.new_auto_routine = new_auto_routine;
		routine_thread = null;
		SmartDashboard.putString("settingAutoRoutine", "SETTING");

	}
	/*** Starts new thread and runs auto routine ***/
	public void start() {
		SmartDashboard.putString("settingAutoRoutine", "STARTING");

		if(routine_thread == null) {
			routine_thread  = new Thread(new Runnable() {
				
				@Override
				public void run() {
						autoTimer.start();
						autoStarted = true;
						SmartDashboard.putString("ThreadSTATE", "startedThread");
						if(new_auto_routine != null) {
							SmartDashboard.putString("settingAutoRoutine", "RUNNING");
							new_auto_routine.run();
						}
				}
			});
			routine_thread.start();
		} else {
			SmartDashboard.putString("settingAutoRoutine", "NULL");
		}
	}
	/*** Stops autoroutine and thread ***/
	public void stop() {
		if(!autoStarted) {
			return;
		} 
		//Stopping for multiple stops
		autoStarted = false;
		if(new_auto_routine != null) {
			new_auto_routine.stop();
		}
		SmartDashboard.putString("ThreadSTATE", "killedThread");
		routine_thread = null;
		autoTimer.stop();
	}
	/*** Gets auto runner timer ***/
	static public Timer getTimer() {
		return autoTimer;
	}

}
