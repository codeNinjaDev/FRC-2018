package org.usfirst.frc.team3997.robot;
import java.lang.Thread;
import java.util.Date;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team3997.robot.auto.Auto;
import org.usfirst.frc.team3997.robot.auto.AutoRoutineRunner;
import org.usfirst.frc.team3997.robot.auto.actions.Action;
import org.usfirst.frc.team3997.robot.auto.actions.DriveIntervalAction;
import org.usfirst.frc.team3997.robot.controllers.ArmController;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.controllers.LightController;
import org.usfirst.frc.team3997.robot.controllers.MotionController;
import org.usfirst.frc.team3997.robot.controllers.VisionController;
import org.usfirst.frc.team3997.robot.feed.DashboardInput;
import org.usfirst.frc.team3997.robot.feed.DashboardLogger;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;
import org.usfirst.frc.team3997.robot.hardware.ControlBoard;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	RobotModel robot;
	RemoteControl humanControl;
	DriveController driveController;
	VisionController visionController;
	LightController lights;
	ArmController armController;

	DashboardLogger dashboardLogger;
	DashboardInput input;

	MotionController motion;

	MasterController masterController;
	Auto auto;
	Timer timer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		/*** Initializes all classes ***/
		robot = new RobotModel();
		humanControl = new ControlBoard();
		driveController = new DriveController(robot, humanControl);
		visionController = new VisionController();
		lights = new LightController();
		dashboardLogger = new DashboardLogger(robot, humanControl);
		input = new DashboardInput();
		motion = new MotionController(robot);
		armController = new ArmController(robot, humanControl);
		masterController = new MasterController(driveController, robot, motion, visionController, lights,
				armController);
		auto = new Auto(masterController);
		timer = new Timer();

		// Sets enabled lights
		lights.setEnabledLights();
		// Resets autonomous
		auto.reset();
		// List autonomous routines on Dashboard
		auto.listOptions();
		// Updates input from Robot Preferences
		input.updateInput();

		/*** Starts camera stream ***/
		new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

			camera.setResolution(640, 480);

			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("Line", 640, 480);

			Mat source = new Mat();
			Mat output = new Mat();

			while (!Thread.interrupted()) {
				cvSink.grabFrame(output);
				int thickness = 2;
				Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);

				Imgproc.line(output, new Point((output.size().width / 2), 0),
						new Point((output.size().width / 2), (output.size().height)), new Scalar(0, 0, 0), thickness);
				outputStream.putFrame(output);
			}
		}).start();
		Thread t = new Thread(() -> {
			ScheduledExecutorService scheduledExecutorService = Executors.newScheduledThreadPool(5);

			Runnable gyroRunnable = new Runnable() {

				public void run() {
					robot.getAngle();
				}
			};

			scheduledExecutorService.scheduleAtFixedRate(gyroRunnable, 0, 20, TimeUnit.MICROSECONDS);
		});
		t.start();
		/*** Update Dashboard ***/

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		/*** Reset all hardware ***/
		robot.reset();
		// Reset auto timer
		AutoRoutineRunner.getTimer().reset();
		// Update robot preferences
		input.updateInput();
		// Stop autonomous before starting
		auto.stop();
		// Reset timer
		timer.reset();
		// Start timer
		timer.start();
		// Starts Autonomous Routine
		auto.start();

	}

	/**
	 * This function is called periodically during autonomous, but is mainly used
	 * for logging
	 */
	@Override
	public void autonomousPeriodic() {
		// Log Gyro angle
		SmartDashboard.putNumber("gyro", robot.getAngle());
		// Start auto pattern on led strip
		lights.setAutoLights();
		// Log data to the Dashboard
		dashboardLogger.updateData();

	}

	/**
	 * This function is called once each time the robot enters tele-operated mode
	 */
	@Override
	public void teleopInit() {
		// Stop autonomous
		auto.stop();
		// Reset Gyro
		robot.resetGyro();
		// Reset hardware timer
		robot.resetTimer();
		// Reset Encoders
		robot.resetEncoders();
		// Reset Drive
		driveController.reset();
		// Reset Arm
		armController.reset();
		// Update input from Robot Preferences
		input.updateInput();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		// Logs arm angle on Dashboard
		SmartDashboard.putNumber("Arm Angle", robot.getArmAngle());
		// Updates Gyro angle
		SmartDashboard.putNumber("gyro", robot.getAngle());
		// Logs data to Dashboard
		dashboardLogger.updateData();
		// Read input from Gamepad
		humanControl.readControls();
		// Updates Drive e.g arcadeDrive
		driveController.update();
		// Updates Intake and arm
		armController.update();
		// Set enabled Light pattern
		lights.setEnabledLights();
		// Log Data to Dashboard

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {

		input.updateInput();

	}

	public void disabledInit() {
		robot.resetGyro();
		// Reset auto timer
		AutoRoutineRunner.getTimer().reset();
		// Reset drive
		driveController.reset();
		// Reset hardware
		robot.reset();
		// Update input from Dashboard
		input.updateInput();
		// Log Data

	}

	public void disabledPeriodic() {

		// Reset auto timer
		AutoRoutineRunner.getTimer().reset();
		// Put Gyro Angle on SmartDashboard
		SmartDashboard.putNumber("gyro", robot.getAngle());
		// Update input from Dashboard
		input.updateInput();
		// Logs arm angle on Dashboard
		SmartDashboard.putNumber("Arm Angle", robot.getArmAngle());
		// Log Dashboard
		dashboardLogger.updateData();
		// Read Gamepad Controlls
		humanControl.readControls();
		// Set Disabled pattern for led strips
		lights.setDisabledLights();
	}

}