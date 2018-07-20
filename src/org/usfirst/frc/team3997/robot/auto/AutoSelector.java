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
import org.usfirst.frc.team3997.robot.auto.routines.RightScale;
import org.usfirst.frc.team3997.robot.auto.routines.RightAutoRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.RightSwitchRightSide;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Gets selected autonomous routine ***/
public class AutoSelector {
	/*** Radio buttons on SmartDashboard ***/
	SendableChooser<CommandGroup> autoChooser;
	/*** Array of All Autonomous Routines ***/
	/*** Selected Routine index ***/
	int selectedIndex;
	
	MasterController controllers;
	/*** registers autonomous routines in order ***/
	public AutoSelector(MasterController controllers) {
		selectedIndex = 0;
		this.controllers = controllers;
		autoChooser = new SendableChooser<CommandGroup>();

		
	} 
	/*** Lists the auto routines on SmartDashboard ***/
	public void listOptions() {
		autoChooser.addDefault("Nothing (Default)", new DoNothingRoutine());
		autoChooser.addObject("Drive (3s)", new DriveThreeSecRoutine(controllers));
		autoChooser.addObject("Pass Auto Line (Drive 100)", new PassAutoLineRoutine(controllers));
		autoChooser.addObject("Custom Routine (check preferences)", new CustomDistanceRoutine(controllers));
		autoChooser.addObject("Left Switch or Scale Auto Routine", new LeftAutoRoutine(controllers));
		autoChooser.addObject("Center Auto Routine", new CenterAutoRoutine(controllers));
		autoChooser.addObject("Right Switch or Scale Auto Routine", new RightAutoRoutine(controllers));
		autoChooser.addObject("Single Cube Center", new OneCubeCenterAutoRoutine(controllers));
		autoChooser.addObject("Left Switch", new LeftSwitchLeftSide(controllers));
		autoChooser.addObject("Rightt Switch", new RightSwitchRightSide(controllers));
		autoChooser.addObject("Left Scale", new LeftScale(controllers));
		autoChooser.addObject("Right Scale", new RightScale(controllers));
		autoChooser.addObject("Motion Profling Routine", new MotionRoutine(controllers));
		
		
	
	}
	
	public CommandGroup getSelectedAuto() {
		return autoChooser.getSelected();
	}
}
