package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.MotionController;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class PassAutoLineRoutine extends AutoRoutine {
	private MasterController controllers;
	Waypoint[] point = new Waypoint[] {
			new Waypoint(0,0,0),
			new Waypoint(10,0,0)
	};
	Trajectory traj;
	public PassAutoLineRoutine(MasterController controllers) {
		this.controllers = controllers;
		
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		traj = MotionController.generateTrajectory(point);
	}

	@Override
	protected void routine() {
		pathFollower(controllers, traj, 10);
	}
}
/*latform: /Linux/arm/
#
# A fatal error has been detected by the Java Runtime Environment:
#
#  SIGSEGV (0xb) at pc=0xab307158, pid=3991, tid=0xb5336470
#
# JRE version: OpenJDK Runtime Environment (8.0_131-b57) (build 1.8.0_131-b57)
# Java VM: OpenJDK Client VM (25.131-b57 mixed mode, Evaluation linux-aarch32 )
# Problematic frame:
# C  [libpathfinderjava.so+0x6158]  pf_trajectory_fromSecondOrderFilter+0x88
#
# Core dump written. Default location: //core or core.3991 (max size 2048 kB). To ensure a full core dump, try "ulimit -c unlimited" before starting Java again
#
# An error report file with more information is saved as:
# /tmp/hs_err_pid3991.log
#
# If you would like to submit a bug report, please visit:
#   http://www.azulsystems.com/support/
# The crash happened outside the Java Virtual Machine in native code.
# See problematic frame for where to report the bug.
#888888888*****/