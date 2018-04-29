package org.usfirst.frc.team3997.robot.auto;

import org.usfirst.frc.team3997.robot.MasterController;
/*** Master autonomous class ***/
public class Auto {
	public AutoRoutine autoRoutine;
	public AutoSelector selector;
	public AutoRoutineRunner runner;
	
	/***Initializes AutoSelector, AutoRoutineRunner, AutoRoutine ***/
	public Auto(MasterController controllers) {
		
		selector = new AutoSelector(controllers);
		runner = new AutoRoutineRunner();
		autoRoutine = selector.getDefaultRoutine();
		
	}
	
	/***Resets auto timer**/
	public void reset() {
		AutoRoutineRunner.getTimer().reset();
		
	}
	/***Lists auto routines on dashboard***/
	public void listOptions() {
		selector.listOptions();
	}
	/*** Runs picked autoRoutine ***/
	public void start() {
		autoRoutine = selector.pick();
		runner.setAutoRoutine(autoRoutine);
		
		autoRoutine.prestart();
		
		runner.start();
	}
	/*** Stops Auto Routine ***/
	public void stop() {
		AutoRoutineRunner.getTimer().reset();
		AutoRoutineRunner.getTimer().stop();
		runner.stop();
		autoRoutine.m_active = false;

	}

}
