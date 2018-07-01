package org.usfirst.frc.team3997.robot.auto;

import java.util.ArrayList;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.routines.CenterAutoRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.CustomDistanceRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.DoNothingRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.DriveThreeSecRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.LeftAutoRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.LeftScale;
import org.usfirst.frc.team3997.robot.auto.routines.LeftSwitchLeftSide;
import org.usfirst.frc.team3997.robot.auto.routines.MotionRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.OneCubeCenterAutoRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.PassAutoLineRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.RighScale;
import org.usfirst.frc.team3997.robot.auto.routines.RightAutoRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.RightSwitchRightSide;
import org.usfirst.frc.team3997.robot.auto.routines.StepVoltageRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.TurnRoutine;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Gets selected autonomous routine ***/
public class AutoSelector {
	/*** Radio buttons on SmartDashboard ***/
	SendableChooser<Integer> autoChooser;
	/*** Array of All Autonomous Routines ***/
	private ArrayList<AutoRoutine> autoRoutines;
	/*** Selected Routine index ***/
	int selectedIndex;
	/*** registers autonomous routines in order ***/
	public AutoSelector(MasterController controllers) {
		selectedIndex = 0;
		autoRoutines = new ArrayList<AutoRoutine>();
		//REMEMBER ORDER SAME ORDER AS LIST OPTIONS!!!
		//
		registerAutonomous(new DoNothingRoutine());
		registerAutonomous(new DriveThreeSecRoutine(controllers));
		registerAutonomous(new PassAutoLineRoutine(controllers));
		registerAutonomous(new TurnRoutine(controllers));
		registerAutonomous(new CustomDistanceRoutine(controllers));
		registerAutonomous(new LeftAutoRoutine(controllers));
		registerAutonomous(new CenterAutoRoutine(controllers));
		registerAutonomous(new RightAutoRoutine(controllers));
		registerAutonomous(new StepVoltageRoutine(controllers));
		registerAutonomous(new OneCubeCenterAutoRoutine(controllers));
		registerAutonomous(new LeftSwitchLeftSide(controllers));
		registerAutonomous(new RightSwitchRightSide(controllers));
		registerAutonomous(new LeftScale(controllers));
		registerAutonomous(new RighScale(controllers));
		registerAutonomous(new MotionRoutine(controllers));


		
	} 
	/*** Lists the auto routines on SmartDashboard ***/
	public void listOptions() {
		autoChooser = new SendableChooser<Integer>();
		autoChooser.addDefault("Nothing (Default)", 0);
		autoChooser.addObject("Drive (3s)", 1);
		autoChooser.addObject("Pass Auto Line (Drive 100)", 2);
		autoChooser.addObject("Turn 90 degrees", 3);
		autoChooser.addObject("Custom Routine (check preferences)", 4);
		autoChooser.addObject("Left Switch or Scale Auto Routine", 5);
		autoChooser.addObject("Center Auto Routine", 6);
		autoChooser.addObject("Right Switch or Scale Auto Routine", 7);
		autoChooser.addObject("Step Voltage Routine", 8);
		autoChooser.addObject("Single Cube Center", 9);
		autoChooser.addObject("Left Switch", 10);
		autoChooser.addObject("Rightt Switch", 11);
		autoChooser.addObject("Left Scale", 12);
		autoChooser.addObject("Right Scale", 13);
		autoChooser.addObject("Motion Profling Routine", 14);
		
		SmartDashboard.putString("AUTO CHOOSER", "TRUE");
		//SmartDashboard.putData("Autonomous: ", autoChooser);
		SmartDashboard.putData("Autonomous", autoChooser);
		System.out.println("AUTO SELECTOR " + autoChooser);
	}
	/*** Sets the autoroutine based on the driver choice ***/
	public AutoRoutine pick() {
		setAutoRoutineByIndex((int)autoChooser.getSelected());
		return getAutoRoutine();
	}
	/*** Appends another autoRoutine ***/
	public void registerAutonomous(AutoRoutine auto) {
		//
		autoRoutines.add(auto);
	}
	/*** gets the selected autonomous routine **/
	public AutoRoutine getAutoRoutine() {
		return autoRoutines.get(selectedIndex);
	}
	/*** sets auto routine ***/
	private void setAutoRoutineByIndex(int input) {
		if(input < 0 || input >= autoRoutines.size()) {
			input = 0;
		}
		selectedIndex = input;
	}
	/*** Gets default routine ***/
	public AutoRoutine getDefaultRoutine() {
		return autoRoutines.get(0);
	}

}
