package org.usfirst.frc.team3997.robot;

import org.usfirst.frc.team3997.robot.feed.DashboardVariables;

public class Params {

	// Params.h: Preferences for the robot
	public static boolean SQUARE_DRIVE_AXIS_INPUT = true;
	public static boolean USE_ARCADE_DRIVE = true;

	public static double GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 1.0;
	public static double GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 1.0;

	public static double X_ARCADE_DRIVE_OUT = 0.0;
	public static double X_ARCADE_DRIVE_LEFT_RIGHT = 6.25;
	public static double X_ARCADE_DRIVE_STRAIGHT = 6.25;
	public static double MAX_SPEED = 1;
	public static double HARDSET_DRIVE_SPEED_MAX = MAX_SPEED;
	//Wheel diameter in inches
	public static double WHEEL_DIAMETER = 6;
	public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	public static double CLIMBER_HARDSET_MOTOR_SPEED = 1.0;

	// ENCODER PARAMS
	public static final double PULSES_PER_ROTATION = 250;

	// DRIVE PID PARAMS

	public static final double DRIVE_Y_PID_VALUES[] = { 1.0, 0.0, 0.0 }; // P,
																			// I,
																			// D
	public static final double DRIVE_Y_PID_SCALE_VALUES[] = { 0.125, 1.0, 1.0 }; // P,
																					// I,
																					// D

	public static final double DRIVE_Y_PID_TOLERANCE = 0.5;
	public static final int DRIVE_Y_PID_SAMPLES_AVERAGE = 1;

	public static final double DRIVE_X_PID_VALUES[] = { 0.0, 0.0, 0.0 }; // P,
																			// I,
																			// D
	public static final double DRIVE_X_PID_SCALE_VALUES[] = { 1.0, 1.0, 1.0 }; // P,
																				// I,
																				// D

	public static final int DRIVE_X_PID_TOLERANCE = 10;
	public static final int DRIVE_X_PID_SAMPLES_AVERAGE = 10;

	// PINI UPDATES
	/*
	 * public static double PINI_P; public static double PINI_D; public static
	 * double PINI_I;
	 */

	// [DEBUGGING]

	// [DRIVE_PID]
	public static double drive_p = 0.4;
	public static double drive_i = 0.0;
	public static double drive_d = 0.05;

	/*// [NEW_DRIVE_PID]
	public static final double new_drive_p = DashboardVariables.DRIVE_P;
	public static final double new_drive_i = DashboardVariables.DRIVE_I;
	public static final double new_drive_d = DashboardVariables.DRIVE_D;
	*/
	// [GEAR_PID]
	public static final double gear_p = DashboardVariables.GEAR_P;
	public static final double gear_i = DashboardVariables.GEAR_I;
	public static final double gear_d = DashboardVariables.GEAR_D;

	public static final double gear_ramp_p = DashboardVariables.GEAR_RAMP_P;
	public static final double gear_ramp_i = DashboardVariables.GEAR_RAMP_I;
	public static final double gear_ramp_d = DashboardVariables.GEAR_RAMP_D;

	// [GEAR_DOWN_PID]
	public static final double gear_down_p = 0.0;
	public static final double gear_down_i = 0.0;
	public static final double gear_down_d = 0.0;

	// [CAMERA]
	public static final double h_low = 0;
	public static final double h_high = 180;
	public static final double s_low = 0;
	public static final double s_high = 255;
	public static final double v_low = 0;
	public static final double v_high = 255;
	
	//[MOTION PROFILING] inches
	public static final double wheel_diameter = .1;
	public static final double maximum_velocity = 3; //TODO add maximum velocity in inches/s
	public static final double maximum_acceleration = 10; //TODO add maximum velocity in inches/s/s
	public static final double maximum_jerk = 30; //TODO add maximum velocity in a/s
    public static final double dt = 0.02; // Calculated - Confirmed

    public static final double kp = 0.8; //1.2;
    public static final double kd = 0.0;
    public static final double ki = 0.0;
    
    public static final double kv = 1.0 / maximum_velocity; // Calculated for test Drivetrain
    public static final double ka = 0.05; //0.015
    //Center of front wheel to center of back wheel
    public static final double wheel_base_width = 0.72;
    //Center of front left wheel to center of front right wheel
    public static final double track_base_width = 0.72;

}
