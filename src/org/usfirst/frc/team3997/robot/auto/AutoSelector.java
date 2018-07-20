package org.usfirst.frc.team3997.robot.auto;

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
import org.usfirst.frc.team3997.robot.auto.routines.RightSwitchRightSide;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
/*** Gets selected autonomous routine ***/
public class AutoSelector {
	/*** Radio buttons on SmartDashboard ***/
	SendableChooser<CommandGroup> autoChooser;
	

	/*** registers autonomous routines in order ***/
	public AutoSelector() {
		
		autoChooser = new SendableChooser<CommandGroup>();


		
	} 
	/*** Lists the auto routines on SmartDashboard ***/
	public void listOptions() {
		autoChooser.addDefault("Nothing (Default)", new DoNothingRoutine());
		autoChooser.addObject("Drive (3s)", new DriveThreeSecRoutine());
		autoChooser.addObject("Pass Auto Line (Drive 100)", new PassAutoLineRoutine());
		autoChooser.addObject("Custom Routine (check preferences)", new CustomDistanceRoutine());
		autoChooser.addObject("Left Switch or Scale Auto Routine", new LeftAutoRoutine());
		autoChooser.addObject("Center Auto Routine", new CenterAutoRoutine());
		autoChooser.addObject("Right Switch or Scale Auto Routine", new CenterAutoRoutine());
		autoChooser.addObject("Single Cube Center", new OneCubeCenterAutoRoutine());
		autoChooser.addObject("Left Switch", new LeftSwitchLeftSide());
		autoChooser.addObject("Right Switch", new RightSwitchRightSide());
		autoChooser.addObject("Left Scale", new LeftScale());
		autoChooser.addObject("Right Scale", new RightScale());
		autoChooser.addObject("Motion Profling Routine", new MotionRoutine());
	}
	
	/*** Get selected Auto ***/
	public CommandGroup getSelectedAuto() {
		return autoChooser.getSelected();
	}
	

}
