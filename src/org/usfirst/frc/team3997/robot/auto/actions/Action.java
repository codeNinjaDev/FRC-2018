package org.usfirst.frc.team3997.robot.auto.actions;
/*** Base class for Actions (Commands for autonomous) ***/
public abstract class Action {
	/*** Returns true if action is finished ***/
	public abstract boolean isFinished();
	/*** Function for code needed to run in loop ***/
	public abstract void update();
	/*** Function for code to run when action is finished ***/
	public abstract void finish();
	/*** Function for code to run when action is starting ***/
	public abstract void start();
	
	
	protected double goal_time;
	protected double start_time;
	protected double x_drive;
	protected double y_drive;

}
