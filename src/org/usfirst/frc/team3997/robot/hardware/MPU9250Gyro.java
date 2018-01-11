/**
 * 
 */
package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;
import java.lang.Math;

/**
 * @author peter
 *
 */
public class MPU9250Gyro extends GyroBase {

	// ==============================================================================
	// ====== Set of useful function to access acceleration. gyroscope,
	// magnetometer,
	// ====== and temperature data
	// ==============================================================================

	// See also MPU-9250 Register Map and Descriptions, Revision 4.0,
	// RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in above
	// document; the MPU9250 and MPU9150 are virtually identical but the latter
	// has
	// a different register map

	/*
	 * MPU9250 Basic Example Code by: Kris Winer date: April 1, 2014 license:
	 * Beerware - Use this code however you'd like. If you find it useful you can
	 * buy me a beer some time.
	 * 
	 * Demonstrate basic MPU-9250 functionality including parameterizing the
	 * register addresses, initializing the sensor, getting properly scaled
	 * accelerometer, gyroscope, and magnetometer data out. Added display functions
	 * to allow display to on breadboard monitor. Addition of 9 DoF sensor fusion
	 * using open source Madgwick and Mahony filter algorithms. Sketch runs on the
	 * 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
	 * 
	 * SDA and SCL should have external pull-up resistors (to 3.3V). 10k resistors
	 * are on the EMSENSR-9250 breakout board.
	 * 
	 * Hardware setup: MPU9250 Breakout --------- Arduino VDD ----------------------
	 * 3.3V VDDI --------------------- 3.3V SDA ----------------------- A4 SCL
	 * ----------------------- A5 GND ---------------------- GND
	 * 
	 * Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. Because
	 * the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V
	 * Teensy 3.1. We have disabled the internal pull-ups used by the Wire library
	 * in the Wire.h/twi.c utility file. We are also using the 400 kHz fast I2C mode
	 * by setting the TWI_FREQ to 400000L /twi.h utility file.
	 */
	// #include "Wire.h"

	// Using NOKIA 5110 monochrome 84 x 48 pixel display
	// pin 9 - System.out clock out (SCLK)
	// pin 8 - System.out data out (DIN)
	// pin 7 - Data/Command select (D/C)
	// pin 5 - LCD chip select (CS)
	// pin 6 - LCD reset (RST)

	// See also MPU-9250 Register Map and Descriptions, Revision 4.0,
	// RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
	// above document; the MPU9250 and MPU9150 are virtually identical but the
	// latter has a different register map
	//
	// Magnetometer Registers
	// Using NOKIA 5110 monochrome 84 x 48 pixel display
	// pin 7 - System.out clock out (SCLK)
	// pin 6 - System.out data out (DIN)
	// pin 5 - Data/Command select (D/C)
	// pin 3 - LCD chip select (SCE)
	// pin 4 - LCD reset (RST)
	I2C i2c;
	// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
	int MS5637_RESET = 0x1E;
	int MS5637_CONVERT_D1 = 0x40;
	int MS5637_CONVERT_D2 = 0x50;
	int MS5637_ADC_READ = 0x00;
	int MS5637_ADDRESS = 0x00;

	// See also MPU-9250 Register Map and Descriptions, Revision 4.0,
	// RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
	// above document; the MPU9250 and MPU9150 are virtually identical but the
	// latter has a different register map
	//
	// Magnetometer Registers
	int AK8963_ADDRESS = 0x0C;
	int AK8963_WHO_AM_I = 0x00; // should return = 0x48
	int AK8963_INFO = 0x01;
	int AK8963_ST1 = 0x02; // data ready status bit 0
	int AK8963_XOUT_L = 0x03; // data
	int AK8963_XOUT_H = 0x04;
	int AK8963_YOUT_L = 0x05;
	int AK8963_YOUT_H = 0x06;
	int AK8963_ZOUT_L = 0x07;
	int AK8963_ZOUT_H = 0x08;
	int AK8963_ST2 = 0x09; // Data overflow bit 3 and data read error status bit 2
	int AK8963_CNTL = 0x0A; // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM
							// (1111) modes on bits 3:0
	int AK8963_ASTC = 0x0C; // Self test control
	int AK8963_I2CDIS = 0x0F; // I2C disable
	int AK8963_ASAX = 0x10; // Fuse ROM x-axis sensitivity adjustment value
	int AK8963_ASAY = 0x11; // Fuse ROM y-axis sensitivity adjustment value
	int AK8963_ASAZ = 0x12; // Fuse ROM z-axis sensitivity adjustment value

	int SELF_TEST_X_GYRO = 0x00;
	int SELF_TEST_Y_GYRO = 0x01;
	int SELF_TEST_Z_GYRO = 0x02;

	/*
	 * int X_FINE_GAIN = 0x03 // [7:0] fine gain int Y_FINE_GAIN = 0x04 int
	 * Z_FINE_GAIN = 0x05 int XA_OFFSET_H = 0x06 // User-defined trim values for
	 * accelerometer int XA_OFFSET_L_TC = 0x07 int YA_OFFSET_H = 0x08 int
	 * YA_OFFSET_L_TC = 0x09 int ZA_OFFSET_H = 0x0A int ZA_OFFSET_L_TC = 0x0B
	 */

	int SELF_TEST_X_ACCEL = 0x0D;
	int SELF_TEST_Y_ACCEL = 0x0E;
	int SELF_TEST_Z_ACCEL = 0x0F;

	int SELF_TEST_A = 0x10;

	int XG_OFFSET_H = 0x13; // User-defined trim values for gyroscope
	int XG_OFFSET_L = 0x14;
	int YG_OFFSET_H = 0x15;
	int YG_OFFSET_L = 0x16;
	int ZG_OFFSET_H = 0x17;
	int ZG_OFFSET_L = 0x18;
	int SMPLRT_DIV = 0x19;
	int CONFIG = 0x1A;
	int GYRO_CONFIG = 0x1B;
	int ACCEL_CONFIG = 0x1C;
	int ACCEL_CONFIG2 = 0x1D;
	int LP_ACCEL_ODR = 0x1E;
	int WOM_THR = 0x1F;

	int MOT_DUR = 0x20; // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB =
						// 1 ms
	int ZMOT_THR = 0x21; // Zero-motion detection threshold bits [7:0]
	int ZRMOT_DUR = 0x22; // Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
							// LSB = 64 ms

	int FIFO_EN = 0x23;
	int I2C_MST_CTRL = 0x24;
	int I2C_SLV0_ADDR = 0x25;
	int I2C_SLV0_REG = 0x26;
	int I2C_SLV0_CTRL = 0x27;
	int I2C_SLV1_ADDR = 0x28;
	int I2C_SLV1_REG = 0x29;
	int I2C_SLV1_CTRL = 0x2A;
	int I2C_SLV2_ADDR = 0x2B;
	int I2C_SLV2_REG = 0x2C;
	int I2C_SLV2_CTRL = 0x2D;
	int I2C_SLV3_ADDR = 0x2E;
	int I2C_SLV3_REG = 0x2F;
	int I2C_SLV3_CTRL = 0x30;
	int I2C_SLV4_ADDR = 0x31;
	int I2C_SLV4_REG = 0x32;
	int I2C_SLV4_DO = 0x33;
	int I2C_SLV4_CTRL = 0x34;
	int I2C_SLV4_DI = 0x35;
	int I2C_MST_STATUS = 0x36;
	int INT_PIN_CFG = 0x37;
	int INT_ENABLE = 0x38;
	int DMP_INT_STATUS = 0x39; // Check DMP interrupt
	int INT_STATUS = 0x3A;
	int ACCEL_XOUT_H = 0x3B;
	int ACCEL_XOUT_L = 0x3C;
	int ACCEL_YOUT_H = 0x3D;
	int ACCEL_YOUT_L = 0x3E;
	int ACCEL_ZOUT_H = 0x3F;
	int ACCEL_ZOUT_L = 0x40;
	int TEMP_OUT_H = 0x41;
	int TEMP_OUT_L = 0x42;
	int GYRO_XOUT_H = 0x43;
	int GYRO_XOUT_L = 0x44;
	int GYRO_YOUT_H = 0x45;
	int GYRO_YOUT_L = 0x46;
	int GYRO_ZOUT_H = 0x47;
	int GYRO_ZOUT_L = 0x48;
	int EXT_SENS_DATA_00 = 0x49;
	int EXT_SENS_DATA_01 = 0x4A;
	int EXT_SENS_DATA_02 = 0x4B;
	int EXT_SENS_DATA_03 = 0x4C;
	int EXT_SENS_DATA_04 = 0x4D;
	int EXT_SENS_DATA_05 = 0x4E;
	int EXT_SENS_DATA_06 = 0x4F;
	int EXT_SENS_DATA_07 = 0x50;
	int EXT_SENS_DATA_08 = 0x51;
	int EXT_SENS_DATA_09 = 0x52;
	int EXT_SENS_DATA_10 = 0x53;
	int EXT_SENS_DATA_11 = 0x54;
	int EXT_SENS_DATA_12 = 0x55;
	int EXT_SENS_DATA_13 = 0x56;
	int EXT_SENS_DATA_14 = 0x57;
	int EXT_SENS_DATA_15 = 0x58;
	int EXT_SENS_DATA_16 = 0x59;
	int EXT_SENS_DATA_17 = 0x5A;
	int EXT_SENS_DATA_18 = 0x5B;
	int EXT_SENS_DATA_19 = 0x5C;
	int EXT_SENS_DATA_20 = 0x5D;
	int EXT_SENS_DATA_21 = 0x5E;
	int EXT_SENS_DATA_22 = 0x5F;
	int EXT_SENS_DATA_23 = 0x60;
	int MOT_DETECT_STATUS = 0x61;
	int I2C_SLV0_DO = 0x63;
	int I2C_SLV1_DO = 0x64;
	int I2C_SLV2_DO = 0x65;
	int I2C_SLV3_DO = 0x66;
	int I2C_MST_DELAY_CTRL = 0x67;
	int SIGNAL_PATH_RESET = 0x68;
	int MOT_DETECT_CTRL = 0x69;
	int USER_CTRL = 0x6A; // Bit 7 enable DMP, bit 3 reset DMP
	int PWR_MGMT_1 = 0x6B; // Device defaults to the SLEEP mode
	int PWR_MGMT_2 = 0x6C;
	int DMP_BANK = 0x6D; // Activates a specific bank in the DMP
	int DMP_RW_PNT = 0x6E; // Set read/write pointer to a specific start address in specified DMP bank
	int DMP_REG = 0x6F; // Register in DMP from which to read or to which to write
	int DMP_REG_1 = 0x70;
	int DMP_REG_2 = 0x71;
	int FIFO_COUNTH = 0x72;
	int FIFO_COUNTL = 0x73;
	int FIFO_R_W = 0x74;
	int WHO_AM_I_MPU9250 = 0x75; // Should return = 0x71
	int XA_OFFSET_H = 0x77;
	int XA_OFFSET_L = 0x78;
	int YA_OFFSET_H = 0x7A;
	int YA_OFFSET_L = 0x7B;
	int ZA_OFFSET_H = 0x7D;
	int ZA_OFFSET_L = 0x7E;
	int MPU9250_ADDRESS = 0x00;

	// Using the MPU9250Teensy 3.1 Add-On shield, ADO is set to 0
	// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
	int ADO = 0;
	
	if(ADO==1)
	{
		MPU9250_ADDRESS = 0x69; // Device address when ADO = 1
		AK8963_ADDRESS = 0x0C; // Address of magnetometer
		MS5637_ADDRESS = 0x76; // Address of altimeter
	}else
	{
		MPU9250_ADDRESS = 0x68; // Device address when ADO = 0
		AK8963_ADDRESS = 0x0C; // Address of magnetometer
		MS5637_ADDRESS = 0x76; // Address of altimeter
	}
	boolean System.outDebug = true; // set to true to get System.out output for debugging

	// Set initial input parameters
	enum Ascale {
		AFS_2G, AFS_4G, AFS_8G, AFS_16G
	};

	enum Gscale {
		GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	};

	enum Mscale {
		MFS_14BITS, // 0.6 mG per LSB
		MFS_16BITS // 0.15 mG per LSB
	};

	int ADC_256 = 0x00; // define pressure and temperature conversion rates
	int ADC_512 = 0x02;
	int ADC_1024 = 0x04;
	int ADC_2048 = 0x06;
	int ADC_4096 = 0x08;
	int ADC_8192 = 0x0A;
	int ADC_D1 = 0x40;
	int ADC_D2 = 0x50;

	// Specify sensor full scale
	int OSR = ADC_8192; // set pressure amd temperature oversample rate
	Gscale gScale = Gscale.GFS_250DPS;
	Ascale aScale = Ascale.AFS_2G;
	Mscale mScale = Mscale.MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
	int mMode = 0x06; // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	double aRes, gRes, mRes; // scale resolutions per LSB for the sensors

	// Pin definitions
	int intPin = 8;
	volatile boolean newData = false;
	boolean newMagData = false;

	int myLed = 13;

	int Pcal[] = new int[8]; // calibration constants from MS5637 PROM registers
	char nCRC; // calculated check sum to ensure PROM integrity
	int D1 = 0;
	int D2 = 0; // raw MS5637 pressure and temperature data
	double dT, OFFSET, SENS, T2, OFFSET2, SENS2; // First order and second order corrections for raw S5637 temperature
													// and pressure data

	int MPU9250Data[] = new int[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
	int accelCount[] = new int[3]; // Stores the 16-bit signed accelerometer sensor output
	int gyroCount[] = new int[3]; // Stores the 16-bit signed gyro sensor output
	int magCount[] = new int[3]; // Stores the 16-bit signed magnetometer sensor output
	double magCalibration[] = { 0, 0, 0 }; // Factory mag calibration and mag bias
	double gyroBias[] = { 0, 0, 0 };
	double accelBias[] = { 0, 0, 0 };
	double magBias[] = { 0, 0, 0 };
	double magScale[] = { 0, 0, 0 }; // Bias corrections for gyro and accelerometer
	int tempCount; // temperature raw count output
	double temperature; // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
	double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature
	double SelfTest[] = new double[6]; // holds results of gyro and accelerometer self test

	// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference
	// System)
	double GyroMeasError = Math.PI * (4.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
	double GyroMeasDrift = Math.PI * (0.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	// There is a tradeoff in the beta parameter between accuracy and response
	// speed.
	// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError
	// of 2.7 degrees/s) was found to give optimal accuracy.
	// However, with this value, the LSM9SD0 response time is about 10 seconds to a
	// stable initial quaternion.
	// Subsequent changes also require a longish lag time to a stable output, not
	// fast enough for a quadcopter or robot car!
	// By increasing beta (GyroMeasError) by about a factor of fifteen, the response
	// time constant is reduced to ~2 sec
	// I haven't noticed any reduction in solution accuracy. This is essentially the
	// I coefficient in a PID control sense;
	// the bigger the feedback coefficient, the faster the solution converges,
	// usually at the expense of accuracy.
	// In any case, this is the free parameter in the Madgwick filtering and fusion
	// scheme.
	double beta = Math.sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
	double zeta = Math.sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick
															// scheme usually set to a small or zero value
	double Kp = 2.0f * 5.0f; // these are the free parameters in the Mahony filter and fusion scheme, Kp for
								// proportional feedback, Ki for integral
	double Ki = 0.0f;

	double delt_t = 0;
	int count = 0;
	int sumCount = 0; // used to control display output rate
	double pitch, yaw, roll;
	double a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler angles and gravity components
	double deltat = 0.0f;
	double sum = 0.0f; // integration interval for both filter schemes
	int lastUpdate = 0;
	int firstUpdate = 0; // used to calculate integration interval
	int Now = 0; // used to calculate integration interval

	double ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
	double lin_ax, lin_ay, lin_az; // linear acceleration (acceleration with gravity component subtracted)
	double q[] = { 1.0f, 0.0f, 0.0f, 0.0f }; // vector to hold quaternion
	double eInt[] = {0.0f, 0.0f, 0.0f}; // vector to hold integral error for Mahony method

	void setup()
	{
	//  Wire.begin();
	//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
	  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
	  // TODO Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
	  Timer.delay(4);
	  
	  // Set up the interrupt pin, its set as active high, push-pull
	  /*pinMode(intPin, INPUT);
	  pinMode(myLed, OUTPUT);
	  digitalWrite(myLed, HIGH);
	  
	  display.begin(); // Initialize the display
	  display.setContrast(40); // Set the contrast*/
	  
	// Start device display with ID of sensor
	  /*display.clearDisplay();
	  display.setTextSize(2);
	  display.setCursor(0,0); display.print("MPU9250");
	  display.setTextSize(1);
	  display.setCursor(0, 20); display.print("9-DOF 16-bit");
	  display.setCursor(0, 30); display.print("motion sensor");
	  display.setCursor(20,40); display.print("60 ug LSB");
	  display.display();*/
	  Timer.delay(1);

	// Set up for data display
	  /*display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
	  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
	  display.clearDisplay();   // clears the screen and buffer*/

	  I2Cscan();// look for I2C devices on the bus
	    
	  // Read the WHO_AM_I register, this is a good test of communication
	  //("MPU9250 9-axis motion sensor...");
	  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
	  /* TODO System.out.print("MPU9250 "); System.out.print("I AM "); System.out.print(c, HEX); System.out.print(" I should be "); System.out.println(0x71, HEX);
	  display.setCursor(20,0); display.print("MPU9250");
	  display.setCursor(0,10); display.print("I AM");
	  display.setCursor(0,20); display.print(c, HEX);  
	  display.setCursor(0,30); display.print("I Should Be");
	  display.setCursor(0,40); display.print(0x71, HEX); 
	  display.display();*/
	  Timer.delay(1); 

	  if (c == 0x71) // WHO_AM_I should always be 0x68
	  {  
	    //System.out.println("MPU9250 is online...");
	    //TODO selftest originally called with one value
	    MPU9250SelfTest(SelfTest, GyroMeasDrift); // Start by performing self test and reporting values
	    /* TODO System.out.print("x-axis self test: acceleration trim within : "); System.out.print(SelfTest[0],1); System.out.println("% of factory value");
	    System.out.print("y-axis self test: acceleration trim within : "); System.out.print(SelfTest[1],1); System.out.println("% of factory value");
	    System.out.print("z-axis self test: acceleration trim within : "); System.out.print(SelfTest[2],1); System.out.println("% of factory value");
	    System.out.print("x-axis self test: gyration trim within : "); System.out.print(SelfTest[3],1); System.out.println("% of factory value");
	    System.out.print("y-axis self test: gyration trim within : "); System.out.print(SelfTest[4],1); System.out.println("% of factory value");
	    System.out.print("z-axis self test: gyration trim within : "); System.out.print(SelfTest[5],1); System.out.println("% of factory value");*/
	    Timer.delay(1);
	    
	   // get sensor resolutions, only need to do this once
	   getAres();
	   getGres();
	   getMres();
	    
	   //System.out.println(" Calibrate gyro and accel");
	   accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	   //System.out.println("accel biases (mg)"); System.out.println(1000.*accelBias[0]); System.out.println(1000.*accelBias[1]); System.out.println(1000.*accelBias[2]);
	   //System.out.println("gyro biases (dps)"); System.out.println(gyroBias[0]); System.out.println(gyroBias[1]); System.out.println(gyroBias[2]);

	  /*display.clearDisplay();
	     
	  display.setCursor(0, 0); display.print("MPU9250 bias");
	  display.setCursor(0, 8); display.print(" x   y   z  ");

	  display.setCursor(0,  16); display.print((int)(1000*accelBias[0])); 
	  display.setCursor(24, 16); display.print((int)(1000*accelBias[1])); 
	  display.setCursor(48, 16); display.print((int)(1000*accelBias[2])); 
	  display.setCursor(72, 16); display.print("mg");
	    
	  display.setCursor(0,  24); display.print(gyroBias[0], 1); 
	  display.setCursor(24, 24); display.print(gyroBias[1], 1); 
	  display.setCursor(48, 24); display.print(gyroBias[2], 1); 
	  display.setCursor(66, 24); display.print("o/s");   
	 
	  display.display();*/
	  Timer.delay(1);  
	   
	  initMPU9250(); 
	  //System.out.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
	  
	  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	  byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
	  //System.out.print("AK8963 "); System.out.print("I AM "); System.out.print(d, HEX); System.out.print(" I should be "); System.out.println(0x48, HEX);
	  /*display.clearDisplay();
	  display.setCursor(20,0); display.print("AK8963");
	  display.setCursor(0,10); display.print("I AM");
	  display.setCursor(0,20); display.print(d, HEX);  
	  display.setCursor(0,30); display.print("I Should Be");
	  display.setCursor(0,40); display.print(0x48, HEX);  
	  display.display();*/
	  Timer.delay(1);
	  
	  // Get magnetometer calibration from AK8963 ROM
	  initAK8963(magCalibration); //System.out.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
	  
	  magcalMPU9250(magBias, magScale);
	  /*System.out.println("AK8963 mag biases (mG)"); System.out.println(magBias[0]); System.out.println(magBias[1]); System.out.println(magBias[2]); 
	  System.out.println("AK8963 mag scale (mG)"); System.out.println(magScale[0]); System.out.println(magScale[1]); System.out.println(magScale[2]);*/ 
	  Timer.delay(2); // add delay to see results before System.out spew of data
	   
	  /*if(System.outDebug) {
	//  System.out.println("Calibration values: ");
	  System.out.print("X-Axis sensitivity adjustment value "); System.out.println(magCalibration[0], 2);
	  System.out.print("Y-Axis sensitivity adjustment value "); System.out.println(magCalibration[1], 2);
	  System.out.print("Z-Axis sensitivity adjustment value "); System.out.println(magCalibration[2], 2);
	  }*/
	  
	  /*display.clearDisplay();
	  display.setCursor(20,0); display.print("AK8963");
	  display.setCursor(0,10); display.print("ASAX "); display.setCursor(50,10); display.print(magCalibration[0], 2);
	  display.setCursor(0,20); display.print("ASAY "); display.setCursor(50,20); display.print(magCalibration[1], 2);
	  display.setCursor(0,30); display.print("ASAZ "); display.setCursor(50,30); display.print(magCalibration[2], 2);
	  display.display();*/
	  Timer.delay(1);  
	  
	  // Reset the MS5637 pressure sensor
	  MS5637Reset();
	  Timer.delay(0.1);
	  //System.out.println("MS5637 pressure sensor reset...");
	  // Read PROM data from MS5637 pressure sensor
	  MS5637PromRead(Pcal);
	  //System.out.println("PROM dta read:");
	  //System.out.print("C0 = "); System.out.println(Pcal[0]);
	  char refCRC = (char) (Pcal[0] >> 12);
	  /*System.out.print("C1 = "); System.out.println(Pcal[1]);
	  System.out.print("C2 = "); System.out.println(Pcal[2]);
	  System.out.print("C3 = "); System.out.println(Pcal[3]);
	  System.out.print("C4 = "); System.out.println(Pcal[4]);
	  System.out.print("C5 = "); System.out.println(Pcal[5]);
	  System.out.print("C6 = "); System.out.println(Pcal[6]);*/
	  
	  nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
	  //System.out.print("Checksum = "); System.out.print(nCRC); System.out.print(" , should be "); System.out.println(refCRC);  
	  
	  /*display.clearDisplay();
	  display.setCursor(20,0); display.print("MS5637");
	  display.setCursor(0,10); display.print("CRC is "); display.setCursor(50,10); display.print(nCRC);
	  display.setCursor(0,20); display.print("Should be "); display.setCursor(50,30); display.print(refCRC);
	  display.display();
	  delay(1000);  */

	  attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for INT pin output of MPU9250

	  }
	  else
	  {
	    //System.out.print("Could not connect to MPU9250: 0x");
	    //System.out.println(c, HEX);
	    while(true) ; // Loop forever if communication doesn't happen
	  }
	}

	void loop()
	{  
	  // If intPin goes high, all data registers have new data
	   if(newData == true) {  // On interrupt, read data
	     newData = false;  // reset newData flag
	     readMPU9250Data(MPU9250Data); // INT cleared on any read
	 //   readAccelData(accelCount);  // Read the x/y/z adc values
	    
	    // Now we'll calculate the accleration value into actual g's
	    ax = (double)MPU9250Data[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
	    ay = (double)MPU9250Data[1]*aRes - accelBias[1];   
	    az = (double)MPU9250Data[2]*aRes - accelBias[2];  
	   
	 //   readGyroData(gyroCount);  // Read the x/y/z adc values

	    // Calculate the gyro value into actual degrees per second
	    gx = (double)MPU9250Data[4]*gRes;  // get actual gyro value, this depends on scale being set
	    gy = (double)MPU9250Data[5]*gRes;  
	    gz = (double)MPU9250Data[6]*gRes;   
	  
	    readMagData(magCount);  // Read the x/y/z adc values
	   
	    // Calculate the magnetometer values in milliGauss
	    // Include factory calibration per data sheet and user environmental corrections
	    if(newMagData == true) {
	      newMagData = false; // reset newMagData flag
	      mx = (double)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
	      my = (double)magCount[1]*mRes*magCalibration[1] - magBias[1];  
	      mz = (double)magCount[2]*mRes*magCalibration[2] - magBias[2];  
	      mx *= magScale[0];
	      my *= magScale[1];
	      mz *= magScale[2]; 
	    } 
	  }
	  
	  Now = (int) Timer.getFPGATimestamp() * 1000;
	  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
	  lastUpdate = Now;

	  sum += deltat; // sum for averaging filter update rate
	  sumCount++;
	  
	  // Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
	  // the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
	  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
	  // For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
	  // we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
	  // positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
	  // function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
	  // This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
	  // Pass gyro rate as rad/s
	    MadgwickQuaternionUpdate(-ax, ay, az, gx*Math.PI/180.0f, -gy*Math.PI/180.0f, -gz*Math.PI/180.0f,  my,  -mx, mz);
	//  if(passThru)MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);

	    // System.out print and/or display at 0.5 s rate independent of data rates
	    delt_t = Timer.getFPGATimestamp() - count;
	    if (delt_t > 500) { // update LCD once per half-second independent of read rate

	    if(System.outDebug) {
	    /*System.out.print("ax = "); System.out.print((int)1000*ax);  
	    System.out.print(" ay = "); System.out.print((int)1000*ay); 
	    System.out.print(" az = "); System.out.print((int)1000*az); System.out.println(" mg");
	    System.out.print("gx = "); System.out.print( gx, 2); 
	    System.out.print(" gy = "); System.out.print( gy, 2); 
	    System.out.print(" gz = "); System.out.print( gz, 2); System.out.println(" deg/s");
	    System.out.print("mx = "); System.out.print( (int)mx ); 
	    System.out.print(" my = "); System.out.print( (int)my ); 
	    System.out.print(" mz = "); System.out.print( (int)mz ); System.out.println(" mG");
	    
	    System.out.print("q0 = "); System.out.print(q[0]);
	    System.out.print(" qx = "); System.out.print(q[1]); 
	    System.out.print(" qy = "); System.out.print(q[2]); 
	    System.out.print(" qz = "); System.out.println(q[3]); */
	    }               
	    tempCount = readTempData();  // Read the gyro adc values
	    temperature = ((double) tempCount) / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
	   // Print temperature in degrees Centigrade      
	    //System.out.print("Gyro temperature is ");  System.out.print(temperature, 1);  System.out.println(" degrees C"); // Print T values to tenths of s degree C
	 
	    D1 = MS5637Read(ADC_D1, OSR);  // get raw pressure value
	    D2 = MS5637Read(ADC_D2, OSR);  // get raw temperature value
	    dT = D2 - Pcal[5]*Math.pow(2,8);    // calculate temperature difference from reference
	    OFFSET = Pcal[2]*Math.pow(2, 17) + dT*Pcal[4]/Math.pow(2,6);
	    SENS = Pcal[1]*Math.pow(2,16) + dT*Pcal[3]/Math.pow(2,7);
	 
	    Temperature = (2000 + (dT*Pcal[6])/Math.pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
	//
	// Second order corrections
	    if(Temperature > 20) 
	    {
	      T2 = 5*dT*dT/Math.pow(2, 38); // correction for high temperatures
	      OFFSET2 = 0;
	      SENS2 = 0;
	    }
	    if(Temperature < 20)                   // correction for low temperature
	    {
	      T2      = 3*dT*dT/Math.pow(2, 33); 
	      OFFSET2 = 61*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
	      SENS2   = 29*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
	    } 
	    if(Temperature < -15)                      // correction for very low temperature
	    {
	      OFFSET2 = OFFSET2 + 17*(100*Temperature + 1500)*(100*Temperature + 1500);
	      SENS2 = SENS2 + 9*(100*Temperature + 1500)*(100*Temperature + 1500);
	    }
	 // End of second order corrections
	 //
	     Temperature = Temperature - T2/100;
	     OFFSET = OFFSET - OFFSET2;
	     SENS = SENS - SENS2;
	 
	     Pressure = (((D1*SENS)/Math.pow(2, 21) - OFFSET)/Math.pow(2, 15))/100;  // Pressure in mbar or kPa
	  
	    final double station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters

	    double baroin = Pressure; // pressure is now in millibars

	    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
	    // comparable to weather report pressure
	    double part1 = baroin - 0.3; //Part 1 of formula
	    final double part2 = 0.0000842288;
	    double part3 = Math.pow(part1, 0.190284);
	    double part4 = (double)station_elevation_m / part3;
	    double part5 = (1.0 + (part2 * part4));
	    double part6 = Math.pow(part5, 5.2553026);
	    double altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
	    baroin = altimeter_setting_pressure_mb * 0.02953;

	    double altitude = 145366.45*(1. - Math.pow((Pressure/1013.25), 0.190284));
	   
	    if(System.outDebug) {
	    /*System.out.print("Digital temperature value = "); System.out.print( (double)Temperature, 2); System.out.println(" C"); // temperature in degrees Celsius
	    System.out.print("Digital temperature value = "); System.out.print(9.*(double) Temperature/5. + 32., 2); System.out.println(" F"); // temperature in degrees Fahrenheit
	    System.out.print("Digital pressure value = "); System.out.print((double) Pressure, 2);  System.out.println(" mbar");// pressure in millibar
	    System.out.print("Altitude = "); System.out.print(altitude, 2); System.out.println(" feet");*/
	    }
	    
	   // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	  // In this coordinate system, the positive z-axis is down toward Earth. 
	  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
	  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	    //Software AHRS:
	 //   yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
	 //   pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
	 //   roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	 //   pitch *= 180.0f / PI;
	 //   yaw   *= 180.0f / PI; 
	 //   yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	 //   if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	 //   roll  *= 180.0f / PI;
	    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
	    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
	    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
	    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	    pitch = -Math.asin(a32);
	    roll  = Math.atan2(a31, a33);
	    yaw   = Math.atan2(a12, a22);
	    pitch *= 180.0f / Math.PI;
	    yaw   *= 180.0f / Math.PI; 
	    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	    roll  *= 180.0f / Math.PI;
	    lin_ax = ax + a31;
	    lin_ay = ay + a32;
	    lin_az = az - a33;
	    /*if(System.outDebug) {
	    System.out.print("Yaw, Pitch, Roll: ");
	    System.out.print(yaw, 2);
	    System.out.print(", ");
	    System.out.print(pitch, 2);
	    System.out.print(", ");
	    System.out.println(roll, 2);

	    System.out.print("Grav_x, Grav_y, Grav_z: ");
	    System.out.print(-a31*1000, 2);
	    System.out.print(", ");
	    System.out.print(-a32*1000, 2);
	    System.out.print(", ");
	    System.out.print(a33*1000, 2);  System.out.println(" mg");
	    System.out.print("Lin_ax, Lin_ay, Lin_az: ");
	    System.out.print(lin_ax*1000, 2);
	    System.out.print(", ");
	    System.out.print(lin_ay*1000, 2);
	    System.out.print(", ");
	    System.out.print(lin_az*1000, 2);  System.out.println(" mg");
	    
	    System.out.print("rate = "); System.out.print((double)sumCount/sum, 2); System.out.println(" Hz");
	    }
	   
	    display.clearDisplay();    
	 
	    display.setCursor(0, 0); display.print(" x   y   z ");

	    display.setCursor(0,  8); display.print((int)(1000*ax)); 
	    display.setCursor(24, 8); display.print((int)(1000*ay)); 
	    display.setCursor(48, 8); display.print((int)(1000*az)); 
	    display.setCursor(72, 8); display.print("mg");
	    
	    display.setCursor(0,  16); display.print((int)(gx)); 
	    display.setCursor(24, 16); display.print((int)(gy)); 
	    display.setCursor(48, 16); display.print((int)(gz)); 
	    display.setCursor(66, 16); display.print("o/s");    

	    display.setCursor(0,  24); display.print((int)(mx)); 
	    display.setCursor(24, 24); display.print((int)(my)); 
	    display.setCursor(48, 24); display.print((int)(mz)); 
	    display.setCursor(72, 24); display.print("mG");    
	 
	    display.setCursor(0,  32); display.print((int)(yaw)); 
	    display.setCursor(24, 32); display.print((int)(pitch)); 
	    display.setCursor(48, 32); display.print((int)(roll)); 
	    display.setCursor(66, 32); display.print("ypr"); */ 
	  
	    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
	    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
	    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
	    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
	    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
	    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
	    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
	    // This filter update rate should be fast enough to maintain accurate platform orientation for 
	    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
	    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
	    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
	    /*display.setCursor(0, 40); display.print(altitude, 0); display.print("ft"); 
	    display.setCursor(68, 0); display.print(9.*Temperature/5. + 32., 0); 
	    display.setCursor(42, 40); display.print((double) sumCount / (1000.*sum), 2); display.print("kHz"); 
	    display.display();*/

	    digitalWrite(myLed, !digitalRead(myLed));
	    count = Timer.getFPGATimestamp(); 
	    sumCount = 0;
	    sum = 0;    
	    }

	}

	// ===================================================================================================================
	// ====== Set of useful function to access acceleration. gyroscope,
	// magnetometer, and temperature data
	// ===================================================================================================================

	void myinthandler() {
		newData = true;
	}

	void getMres() {
		switch (mScale) {
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		case MFS_14BITS:
			mRes = 10. * 4912. / 8190.; // Proper scale to return milliGauss
			break;
		case MFS_16BITS:
			mRes = 10. * 4912. / 32760.0; // Proper scale to return milliGauss
			break;
		}
	}

	void getGres() {
		switch (gScale) {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
		// value:
		case GFS_250DPS:
			gRes = 250.0 / 32768.0;
			break;
		case GFS_500DPS:
			gRes = 500.0 / 32768.0;
			break;
		case GFS_1000DPS:
			gRes = 1000.0 / 32768.0;
			break;
		case GFS_2000DPS:
			gRes = 2000.0 / 32768.0;
			break;
		}
	}

	void getAres() {
		switch (aScale) {
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
		// value:
		case AFS_2G:
			aRes = 2.0 / 32768.0;
			break;
		case AFS_4G:
			aRes = 4.0 / 32768.0;
			break;
		case AFS_8G:
			aRes = 8.0 / 32768.0;
			break;
		case AFS_16G:
			aRes = 16.0 / 32768.0;
			break;
		}
	}

	void readMPU9250Data(int destination[])
	{
	  int rawData[] = new int[14];  // x/y/z accel register data stored here
	  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
	  destination[0] = ((int)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	  destination[1] = ((int)rawData[2] << 8) | rawData[3] ;  
	  destination[2] = ((int)rawData[4] << 8) | rawData[5] ; 
	  destination[3] = ((int)rawData[6] << 8) | rawData[7] ;   
	  destination[4] = ((int)rawData[8] << 8) | rawData[9] ;  
	  destination[5] = ((int)rawData[10] << 8) | rawData[11] ;  
	  destination[6] = ((int)rawData[12] << 8) | rawData[13] ; 
	}

	void readAccelData(int destination[])
	{
	  int rawData[] = new int[6];  // x/y/z accel register data stored here
	  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	  destination[0] = ((int)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	  destination[1] = ((int)rawData[2] << 8) | rawData[3] ;  
	  destination[2] = ((int)rawData[4] << 8) | rawData[5] ; 
	}

	void readGyroData(int destination[])
	{
	  int rawData[] = new int[6];  // x/y/z gyro register data stored here
	  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	  destination[0] = ((int)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	  destination[1] = ((int)rawData[2] << 8) | rawData[3] ;  
	  destination[2] = ((int)rawData[4] << 8) | rawData[5] ; 
	}

	void readMagData(int destination[])
	{
	  int rawData[] = new int[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	  newMagData = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
	  if(newMagData == true) { // wait for magnetometer data ready bit to be set
	  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
	  int c = rawData[6]; // End data read by reading ST2 register
	    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
	    destination[0] = ((int)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
	    destination[1] = ((int)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
	    destination[2] = ((int)rawData[5] << 8) | rawData[4] ; 
	   }
	  }
	}

	int readTempData()
	{
	  int rawData[] = new int[2];  // x/y/z gyro register data stored here
	  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
	  return ((int)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
	}

	void initAK8963(double destination[])
	{
	  // First extract the factory calibration for each magnetometer axis
	  int rawData[] = new int[3];  // x/y/z gyro calibration data stored here
	  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	  Timer.delay(.01);
	  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	  Timer.delay(.01);
	  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	  destination[0] =  (double)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	  destination[1] =  (double)(rawData[1] - 128)/256. + 1.;  
	  destination[2] =  (double)(rawData[2] - 128)/256. + 1.; 
	  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	  Timer.delay(.01);
	  // Configure the magnetometer for continuous read and highest resolution
	  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	  // and enable continuous mode data acquisition mMode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | mMode); // Set magnetometer data resolution and sample ODR
	  Timer.delay(.01);
	}

	void initMPU9250() {
		// wake up device
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
		  Timer.delay(.1); // Wait for all registers to reset

		// get stable time source
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready
														// else
		  Timer.delay(.2); 

		// Configure Gyro and Thermometer
		// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
		// respectively;
		// minimum delay time for this setting is 5.9 ms, which means sensor fusion
		// update rates cannot
		// be higher than 1 / 0.0059 = 170 Hz
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
		// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8
		// kHz, or 1 kHz
		writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

		// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update
														// rate
		// determined inset in CONFIG above

		// Set gyroscope full scale range
		// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted
		// into positions 4:3
		int c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x03; // Clear Fchoice bits [1:0]
		c = c & ~0x18; // Clear GFS bits [4:3]
		c = c | Gscale << 3; // Set full scale range for the gyro
		// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits
		// 1:0 of GYRO_CONFIG
		writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

		// Set accelerometer full-scale range configuration
		c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x18; // Clear AFS bits [4:3]
		c = c | Ascale << 3; // Set full scale range for the accelerometer
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

		// Set accelerometer sample rate configuration
		// It is possible to get a 4 kHz sample rate from the accelerometer by choosing
		// 1 for
		// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
		c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
		c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
		c = c | 0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

		// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
		// but all these rates are further reduced by a factor of 5 to 200 Hz because of
		// the SMPLRT_DIV setting

		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until
		// interrupt cleared,
		// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
		// can join the I2C bus and all can be controlled by the Arduino as master
		// writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
		writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12); // INT is 50 microsecond pulse and any read to clear
		writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
		  Timer.delay(.1); 
	}

	// Function which accumulates gyro and accelerometer data after device
	// initialization. It calculates the average
	// of the at-rest readings and then loads the resulting offsets into
	// accelerometer and gyro bias registers.
	void accelgyrocalMPU9250(double dest1[], double dest2[])
	{  
	  int data[] = new int[12]; // data array to hold accelerometer and gyro x, y, z, data
	  int ii, packet_count, fifo_count;
	  int gyro_bias[]  = {0, 0, 0};
	  int accel_bias[] = {0, 0, 0};
	  
	 // reset device
	  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	  Timer.delay(.1); 
	   
	 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	 // else use the internal oscillator, bits 2:0 = 001
	  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
	  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	  Timer.delay(.2); 

	// Configure device for bias calculation
	  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	  Timer.delay(.015); 
	  
	// Configure MPU6050 gyro and accelerometer for bias calculation
	  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
	 
	  int  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	  int  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
	  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	  Timer.delay(.04);  // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	  fifo_count = ((int)data[0] << 8) | data[1];
	  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	  
	  for (ii = 0; ii < packet_count; ii++) {
	    int accel_temp[] = {0, 0, 0};
	    int gyro_temp[] = {0, 0, 0};
	    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
	    accel_temp[0] = (int) (((int)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
	    accel_temp[1] = (int) (((int)data[2] << 8) | data[3]  ) ;
	    accel_temp[2] = (int) (((int)data[4] << 8) | data[5]  ) ;    
	    gyro_temp[0]  = (int) (((int)data[6] << 8) | data[7]  ) ;
	    gyro_temp[1]  = (int) (((int)data[8] << 8) | data[9]  ) ;
	    gyro_temp[2]  = (int) (((int)data[10] << 8) | data[11]) ;
	    
	    accel_bias[0] += (int) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	    accel_bias[1] += (int) accel_temp[1];
	    accel_bias[2] += (int) accel_temp[2];
	    gyro_bias[0]  += (int) gyro_temp[0];
	    gyro_bias[1]  += (int) gyro_temp[1];
	    gyro_bias[2]  += (int) gyro_temp[2];
	            
	}
	    accel_bias[0] /= (int) packet_count; // Normalize sums to get average count biases
	    accel_bias[1] /= (int) packet_count;
	    accel_bias[2] /= (int) packet_count;
	    gyro_bias[0]  /= (int) packet_count;
	    gyro_bias[1]  /= (int) packet_count;
	    gyro_bias[2]  /= (int) packet_count;
	    
	  if(accel_bias[2] > 0L) {accel_bias[2] -= (int) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	  else {accel_bias[2] += (int) accelsensitivity;}
	   
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	  data[3] = (-gyro_bias[1]/4)       & 0xFF;
	  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	  data[5] = (-gyro_bias[2]/4)       & 0xFF;
	  
	// Push gyro biases to hardware registers
	  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
	  
	// Output scaled gyro biases for display in the main program
	  dest1[0] = (double) gyro_bias[0]/(double) gyrosensitivity;  
	  dest1[1] = (double) gyro_bias[1]/(double) gyrosensitivity;
	  dest1[2] = (double) gyro_bias[2]/(double) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	  int accel_bias_reg[] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	  accel_bias_reg[0] = (int) (((int)data[0] << 8) | data[1]);
	  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	  accel_bias_reg[1] = (int) (((int)data[0] << 8) | data[1]);
	  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	  accel_bias_reg[2] = (int) (((int)data[0] << 8) | data[1]);
	  
	  int mask = 1; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	  int mask_bit[] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
	  
	  for(ii = 0; ii < 3; ii++) {
		
	    if(((accel_bias_reg[ii] & mask) == 1)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	  }
	  
	  // Construct total accelerometer bias, including calculated average accelerometer bias from above
	  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	  accel_bias_reg[1] -= (accel_bias[1]/8);
	  accel_bias_reg[2] -= (accel_bias[2]/8);
	  
	  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	  data[1] = (accel_bias_reg[0])      & 0xFF;
	  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	  data[3] = (accel_bias_reg[1])      & 0xFF;
	  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	  data[5] = (accel_bias_reg[2])      & 0xFF;
	  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	 
	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	*/
	// Output scaled accelerometer biases for display in the main program
	   dest2[0] = (double)accel_bias[0]/(double)accelsensitivity; 
	   dest2[1] = (double)accel_bias[1]/(double)accelsensitivity;
	   dest2[2] = (double)accel_bias[2]/(double)accelsensitivity;
	}

	void magcalMPU9250(double dest1[], double dest2[]) 
	{
	  int ii = 0, sample_count = 0;
	  int mag_bias[] = {0, 0, 0}, mag_scale[] = {0, 0, 0};
	  int mag_max[] = {-32767, -32767, -32767}, mag_min[] = {32767, 32767, 32767}, mag_temp[] = {0, 0, 0};

	  //System.out.println("Mag Calibration: Wave device in a figure eight until done!");
	  Timer.delay(4);
	  
	    // shoot for ~fifteen seconds of mag data
	    if(mMode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	    if(mMode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
	   for(ii = 0; ii < sample_count; ii++) {
	    readMagData(mag_temp);  // Read the mag data   
	    for (int jj = 0; jj < 3; jj++) {
	      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
	      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
	    }
	    if(mMode == 0x02) Timer.delay(.135);  // at 8 Hz ODR, new mag data is available every 125 ms
	    if(mMode == 0x06) Timer.delay(.012);  // at 100 Hz ODR, new mag data is available every 10 ms
	    }

//	    System.out.println("mag x min/max:"); System.out.println(mag_max[0]); System.out.println(mag_min[0]);
//	    System.out.println("mag y min/max:"); System.out.println(mag_max[1]); System.out.println(mag_min[1]);
//	    System.out.println("mag z min/max:"); System.out.println(mag_max[2]); System.out.println(mag_min[2]);

	    // Get hard iron correction
	    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
	    
	    dest1[0] = (double) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
	    dest1[1] = (double) mag_bias[1]*mRes*magCalibration[1];   
	    dest1[2] = (double) mag_bias[2]*mRes*magCalibration[2];  
	       
	    // Get soft iron correction estimate
	    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	    double avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	    avg_rad /= 3.0;

	    dest2[0] = avg_rad/((double)mag_scale[0]);
	    dest2[1] = avg_rad/((double)mag_scale[1]);
	    dest2[2] = avg_rad/((double)mag_scale[2]);
	  
	   //System.out.println("Mag Calibration done!");
	}

	// Accelerometer and gyroscope self test; check calibration wrt factory settings
	void MPU9250SelfTest(double[] selfTest2, double destination[]) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
	{
	   int rawData[] = {0, 0, 0, 0, 0, 0};
	   int selfTest[];
	   int gAvg[] = {0, 0, 0};
	   int aAvg[] = {0, 0, 0};
	   int aSTAvg[] = {0, 0, 0};
	   int gSTAvg[] = {0, 0, 0};
	   double factoryTrim[] = new double[6];
	   int FS = 0;
	   
	  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

	  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
	  
	  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
	  aAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  aAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;  
	  aAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ; 
	  
	    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
	  gAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  gAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;  
	  gAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ; 
	  }
	  
	  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
	  aAvg[ii] /= 200;
	  gAvg[ii] /= 200;
	  }
	  
	// Configure the accelerometer for self-test
	   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	   Timer.delay(.025);  // Delay a while to let the device stabilize

	  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
	  
	  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	  aSTAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  aSTAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;  
	  aSTAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ; 
	  
	    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	  gSTAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  gSTAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]) ;  
	  gSTAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]) ; 
	  }
	  
	  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
	  aSTAvg[ii] /= 200;
	  gSTAvg[ii] /= 200;
	  }   
	  
	 // Configure the gyro and accelerometer for normal operation
	   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
	   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
	   Timer.delay(.025);  // Delay a while to let the device stabilize
	   
	   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	   selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	   selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	   selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	   selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	   selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	   selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

	  // Retrieve factory self-test value from self-test code reads
	   factoryTrim[0] = (double)(2620/1<<FS)*(Math.pow( 1.01 , ((double)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	   factoryTrim[1] = (double)(2620/1<<FS)*(Math.pow( 1.01 , ((double)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	   factoryTrim[2] = (double)(2620/1<<FS)*(Math.pow( 1.01 , ((double)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	   factoryTrim[3] = (double)(2620/1<<FS)*(Math.pow( 1.01 , ((double)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	   factoryTrim[4] = (double)(2620/1<<FS)*(Math.pow( 1.01 , ((double)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	   factoryTrim[5] = (double)(2620/1<<FS)*(Math.pow( 1.01 , ((double)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
	 
	 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	 // To get percent, must multiply by 100
	   for (int i = 0; i < 3; i++) {
	     destination[i]   = 100.0*((double)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
	     destination[i+3] = 100.0*((double)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
	   }
	   
	}

	// I2C communication with the MS5637 is a little different from that with the
	// MPU9250 and most other sensors
	// For the MS5637, we write commands, and the MS5637 sends data in response,
	// rather than directly reading
	// MS5637 registers

	void MS5637Reset() {
		
		//Wire.beginTransmission(MS5637_ADDRESS); // Initialize the Tx buffer
		i2c.write(MS5637_ADDRESS, MS5637_RESET);
		//Wire.write(MS5637_RESET); // Put reset command in Tx buffer
		//Wire.endTransmission(); // Send the Tx buffer
	}

	void MS5637PromRead(int destination[])
	        {
	        int data[] = {0,0};
	        for (int ii = 0; ii < 7; ii++) {
	        	
	          //Wire.beginTransmission(MS5637_ADDRESS); 
	  		i2c.write(MS5637_ADDRESS, MS5637_RESET);
// Initialize the Tx buffer
	          i2c.write(MS5637_ADDRESS, (0xA0 | ii << 1));   
	          // Put PROM address in Tx buffer
	          //Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
		  int i = 0;
	      //i2c.read(MS5637_ADDRESS, 2, buffer)    
		  Wire.requestFrom(MS5637_ADDRESS, 2);   // Read two bytes from slave PROM address 
		  while (Wire.available()) {
	          data[i++] = Wire.read(); }               // Put read results in the Rx buffer
	          destination[ii] = (int) (((int) data[0] << 8) | data[1]); // construct PROM data for return to main program
	        }
	        }

	int MS5637Read(int CMD, int OSR)  // temperature data read
	        {
	        int data[] = {0,0,0};
	        //Wire.beginTransmission(MS5637_ADDRESS); // Initialize the Tx buffer
	        i2c.write(MS5637_ADDRESS, CMD | OSR); // Put pressure conversion command in Tx buffer
	        //Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
	        
	        switch (OSR)
	        {
	          case ADC_256: Timer.delay(.001); break;  // delay for conversion to complete
	          case ADC_512: Timer.delay(.003); break;
	          case ADC_1024: Timer.delay(.004); break;
	          case ADC_2048: Timer.delay(.006); break;
	          case ADC_4096: Timer.delay(.01); break;
	          case ADC_8192: Timer.delay(.02); break;
	        }
	       
	        //Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
	        i2c.write(MS5637_ADDRESS, 0x00);                        // Put ADC read command in Tx buffer
	        //Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
		int i = 0;
	        Wire.requestFrom(MS5637_ADDRESS, 3);     // Read three bytes from slave PROM address 
		while (Wire.available()) {
	        data[i++] = Wire.read(); }               // Put read results in the Rx buffer
	        return (int) (((int) data[0] << 16) | (int) data[1] << 8 | data[2]); // construct PROM data for return to main program
	        }

	char MS5637checkCRC(int n_prom)  // calculate checksum from PROM register contents
	{
	  int cnt;
	  int n_rem = 0;
	  char n_bit;
	  
	  n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
	  n_prom[7] = 0;
	  for(cnt = 0; cnt < 16; cnt++)
	  {
	    if(cnt%2==1) n_rem ^= (short) ((n_prom[cnt>>1]) & 0x00FF);
	    else         n_rem ^= (short)  (n_prom[cnt>>1]>>8);
	    for(n_bit = 8; n_bit > 0; n_bit--)
	    {
	        if(n_rem & 0x8000)    n_rem = (n_rem<<1) ^ 0x3000;
	        else                  n_rem = (n_rem<<1);
	    }
	  }
	  n_rem = ((n_rem>>12) & 0x000F);
	  return (n_rem ^ 0x00);
	}

	// I2C scan function

	void I2Cscan() {
		// scan for i2c devices
		byte error, address;
		int nDevices;

		System.out.println("Scanning...");

		nDevices = 0;
		for (address = 1; address < 127; address++) {
			// The i2c_scanner uses the return value of
			// the Write.endTransmisstion to see if
			// a device did acknowledge to the address.
			Wire.beginTransmission(address);
			error = Wire.endTransmission();

			if (error == 0) {
				System.out.print("I2C device found at address 0x");
				if (address < 16)
					System.out.print("0");
				System.out.print(address, HEX);
				System.out.println("  !");

				nDevices++;
			} else if (error == 4) {
				System.out.print("Unknow error at address 0x");
				if (address < 16)
					System.out.print("0");
				System.out.println(address, HEX);
			}
		}
		if (nDevices == 0)
			System.out.println("No I2C devices found\n");
		else
			System.out.println("done\n");

	}

	// I2C read/write functions for the MPU9250 and AK8963 sensors

	void writeByte(int address, int subAddress, int data) {
		
		Wire.beginTransmission(address); // Initialize the Tx buffer
		Wire.write(subAddress); // Put slave register address in Tx buffer
		Wire.write(data); // Put data in Tx buffer
		Wire.endTransmission(); // Send the Tx buffer
	}

	byte readByte(int address, int subAddress) {
		int data; // `data` will store the register data
		Wire.beginTransmission(address); // Initialize the Tx buffer
		Wire.write(subAddress); // Put slave register address in Tx buffer
		Wire.endTransmission(I2C_NOSTOP); // Send the Tx buffer, but send a restart to keep connection alive
		// Wire.endTransmission(false); // Send the Tx buffer, but send a restart to
		// keep connection alive
		// Wire.requestFrom(address, 1); // Read one byte from slave register address
		Wire.requestFrom(address, (size_t) 1); // Read one byte from slave register address
		data = Wire.read(); // Fill Rx buffer with result
		return data; // Return data read from slave register
	}

	void readBytes(int address, int subAddress, int count, int dest) {
		Wire.beginTransmission(address); // Initialize the Tx buffer
		Wire.write(subAddress); // Put slave register address in Tx buffer
		Wire.endTransmission(I2C_NOSTOP); // Send the Tx buffer, but send a restart to keep connection alive
		// Wire.endTransmission(false); // Send the Tx buffer, but send a restart to
		// keep connection alive
		int i = 0;
		// Wire.requestFrom(address, count); // Read bytes from slave register address
		Wire.requestFrom(address, (size_t) count); // Read bytes from slave register address
		while (Wire.available()) {
			dest[i++] = Wire.read();
		} // Put read results in the Rx buffer
	}
	
	public MPU9250Gyro() {
		i2c = new I2C(Port.kOnboard, MPU9250_ADDRESS)
	}
}
