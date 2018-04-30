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
	
	/*** Goal end time for autonomous action ***/
	protected double goal_time;
	/** Start time for autonomous ***/
	protected double start_time;
	/*** Rotation value for autonomous action ***/
	protected double x_drive;
	/*** Forward value for autonomous action ***/
	protected double y_drive;

}
