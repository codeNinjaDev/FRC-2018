package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

public class StepVoltageRoutine extends AutoRoutine {
	RobotModel robot;
	public StepVoltageRoutine(MasterController controllers) {
		robot = controllers.getRobotModel();
	}
	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void routine() {

		/*robot.setVoltage(12);
		waitTime(10);
		robot.setVoltage(10);
		waitTime(10);
		robot.setVoltage(8);
		waitTime(10);
		robot.setVoltage(6);
		waitTime(10);
		robot.setVoltage(4);
		waitTime(10);*/
	}

}
