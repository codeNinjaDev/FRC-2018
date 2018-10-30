/**
 * 
 */
package org.usfirst.frc.team3997.robot.hardware;

import org.usfirst.frc.team3997.robot.Params;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * MPU9250 Gyro class
 * 
 * @author peter
 * 
 */
public class MPU9250Gyro extends GyroBase implements PIDSource {

	//Magnetometer Registers
	int AK8963_ADDRESS   = 0x0C;
	int AK8963_WHO_AM_I  = 0x00; // should return = 0x48
	int AK8963_INFO      = 0x01;
	int AK8963_ST1       = 0x02;  // data ready status bit 0
	int AK8963_XOUT_L   = 0x03;  // data
	int AK8963_XOUT_H  = 0x04;
	int AK8963_YOUT_L  = 0x05;
	int AK8963_YOUT_H  = 0x06;
	int AK8963_ZOUT_L  = 0x07;
	int AK8963_ZOUT_H  = 0x08;
	int AK8963_ST2       = 0x09;  // Data overflow bit 3 and data read error status bit 2
	int AK8963_CNTL      = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
	int AK8963_ASTC      = 0x0C;  // Self test control
	int AK8963_I2CDIS    = 0x0F;  // I2C disable
	int AK8963_ASAX      = 0x10;  // Fuse ROM x-axis sensitivity adjustment value
	int AK8963_ASAY      = 0x11;  // Fuse ROM y-axis sensitivity adjustment value
	int AK8963_ASAZ      = 0x12;  // Fuse ROM z-axis sensitivity adjustment value

	int SELF_TEST_X_GYRO = 0x00;
	int SELF_TEST_Y_GYRO = 0x01;
	int SELF_TEST_Z_GYRO = 0x02;

	/*int X_FINE_GAIN      = 0x03 // [7:0] fine gain
	int Y_FINE_GAIN      = 0x04
	int Z_FINE_GAIN      = 0x05
	int XA_OFFSET_H      = 0x06 // User-defined trim values for accelerometer
	int XA_OFFSET_L_TC   = 0x07
	int YA_OFFSET_H      = 0x08
	int YA_OFFSET_L_TC   = 0x09
	int ZA_OFFSET_H      = 0x0A
	int ZA_OFFSET_L_TC   = 0x0B */

	int SELF_TEST_X_ACCEL = 0x0D;
	int SELF_TEST_Y_ACCEL = 0x0E;
	int SELF_TEST_Z_ACCEL = 0x0F;

	int SELF_TEST_A      = 0x10;


	int XG_OFFSET_H      = 0x13;  // User-defined trim values for gyroscope
	int XG_OFFSET_L      = 0x14;
	int YG_OFFSET_H      = 0x15;
	int YG_OFFSET_L      = 0x16;
	int ZG_OFFSET_H      = 0x17;
	int ZG_OFFSET_L      = 0x18;
	int SMPLRT_DIV       = 0x19;
	int CONFIG           = 0x1A;
	int GYRO_CONFIG      = 0x1B;
	int ACCEL_CONFIG     = 0x1C;
	int ACCEL_CONFIG2    = 0x1D;
	int LP_ACCEL_ODR     = 0x1E;
	int WOM_THR          = 0x1F;

	int MOT_DUR          = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
	int ZMOT_THR         = 0x21;  // Zero-motion detection threshold bits [7:0]
	int ZRMOT_DUR        = 0x22; // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

	int FIFO_EN          = 0x23;
	int I2C_MST_CTRL     = 0x24;
	int I2C_SLV0_ADDR    = 0x25;
	int I2C_SLV0_REG     = 0x26;
	int I2C_SLV0_CTRL    = 0x27;
	int I2C_SLV1_ADDR    = 0x28;
	int I2C_SLV1_REG     = 0x29;
	int I2C_SLV1_CTRL    = 0x2A;
	int I2C_SLV2_ADDR    = 0x2B;
	int I2C_SLV2_REG     = 0x2C;
	int I2C_SLV2_CTRL    = 0x2D;
	int I2C_SLV3_ADDR    = 0x2E;
	int I2C_SLV3_REG     = 0x2F;
	int I2C_SLV3_CTRL    = 0x30;
	int I2C_SLV4_ADDR    = 0x31;
	int I2C_SLV4_REG     = 0x32;
	int I2C_SLV4_DO      = 0x33;
	int I2C_SLV4_CTRL    = 0x34;
	int I2C_SLV4_DI      = 0x35;
	int I2C_MST_STATUS   = 0x36;
	int INT_PIN_CFG      = 0x37;
	int INT_ENABLE       = 0x38;
	int DMP_INT_STATUS   = 0x39; // Check DMP interrupt
	int INT_STATUS       = 0x3A;
	int ACCEL_XOUT_H     = 0x3B;
	int ACCEL_XOUT_L     = 0x3C;
	int ACCEL_YOUT_H     = 0x3D;
	int ACCEL_YOUT_L     = 0x3E;
	int ACCEL_ZOUT_H     = 0x3F;
	int ACCEL_ZOUT_L     = 0x40;
	int TEMP_OUT_H       = 0x41;
	int TEMP_OUT_L       = 0x42;
	int GYRO_XOUT_H      = 0x43;
	int GYRO_XOUT_L      = 0x44;
	int GYRO_YOUT_H      = 0x45;
	int GYRO_YOUT_L      = 0x46;
	int GYRO_ZOUT_H      = 0x47;
	int GYRO_ZOUT_L      = 0x48;
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
	int I2C_SLV0_DO      = 0x63;
	int I2C_SLV1_DO      = 0x64;
	int I2C_SLV2_DO      = 0x65;
	int I2C_SLV3_DO      = 0x66;
	int I2C_MST_DELAY_CTRL = 0x67;
	int SIGNAL_PATH_RESET  = 0x68;
	int MOT_DETECT_CTRL  = 0x69;
	int USER_CTRL        = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP
	int PWR_MGMT_1       = 0x6B; // Device defaults to the SLEEP mode
	int PWR_MGMT_2       = 0x6C;
	int DMP_BANK         = 0x6D;  // Activates a specific bank in the DMP
	int DMP_RW_PNT       = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank
	int DMP_REG          = 0x6F;  // Register in DMP from which to read or to which to write
	int DMP_REG_1        = 0x70;
	int DMP_REG_2        = 0x71;
	int FIFO_COUNTH      = 0x72;
	int FIFO_COUNTL      = 0x73;
	int FIFO_R_W         = 0x74;
	int WHO_AM_I_MPU9250 = 0x75; // Should return = 0x71
	int XA_OFFSET_H      = 0x77;
	int XA_OFFSET_L      = 0x78;
	int YA_OFFSET_H      = 0x7A;
	int YA_OFFSET_L      = 0x7B;
	int ZA_OFFSET_H      = 0x7D;
	int ZA_OFFSET_L      = 0x7E;

	int MPU9250_ADDRESS = 0x68;  // Device address when ADO = 0

	int ADC_256 = 0x00; // define pressure and temperature conversion rates
	int ADC_512 = 0x02;
	int ADC_1024 = 0x04;
	int ADC_2048 = 0x06;
	int ADC_4096 = 0x08;
	int ADC_8192 = 0x0A;
	int ADC_D1 = 0x40;
	int ADC_D2 = 0x50;

	double gRes = 0;
	double aRes = 0;
	// Using the MPU9250Teensy 3.1 Add-On shield, ADO is set to 0
	// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
	int ADO = 0;
	int[] gyroCount = new int[3];
	int[] gyroData = new int[3];
	double[] gyroBias = new double[3];
	double eInt[] = new double[3];
	int[] accelCount = new int[3];
	int[] accelData = new int[3];
	double[] accelBias = new double[3];
	
	int[] magCount = new int[3];
	double[] magBias = new double[3];
	double[] magCalibration = new double[3];
	double mRes =0;

	boolean SystemDebug = true; // set to true to get System.out output for debugging


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
	// Yaw is side to side rotation, most important // scheme usually set to a small
	// or zero 
	double pastTime;
	double yaw, pitch, roll;
	int delt_t = 0, count = 0, sumCount = 0; // used to control display output rate
	double a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler angles and gravity components
	double deltat = 0.0;
	double sum = 0.0; // integration interval for both filter schemes
	int lastUpdate = 0;
	int firstUpdate = 0; // used to calculate integration interval
	int Now = 0; // used to calculate integration interval
	double Kp = 5 * 2;
	double Ki = 0;
	/*** Rate of change of gyro (I think) ***/
	double gx, gy, gz, ax, ay, az, mx, my, mz; // variables to hold latest sensor data values
	// Set initial input parameters
	double currentTime = 0;
	double lastTime = 0;
	//TODO WHERE DOES THIS COME FROM?
	final int GFS_250DPS = 0x00;
	final int GFS_500DPS = 0x01;
	final int GFS_1000DPS = 0x02;
	final int GFS_2000DPS = 0x03;

	final int AFS_2G = 0x00;
	final int AFS_4G = 0x01;
	final int AFS_8G = 0x02;
	final int AFS_16G = 0x03;

	final int MFS_14BITS = 0x00;
	final int MFS_16BITS = 0x01;
	final int Mmode = 0x02;

	final int NUM_SAMPLES = 10;
	double[] gyroSamples = new double[NUM_SAMPLES];
	int tempCount;
	double temperature;
	double offset = 0;
	double q[] = {1, 0, 0, 0};
	I2C i2c;
	I2C magI2C;
	PIDSourceType pidSourceType;
	int gScale;
	int aScale;
	int mScale;

	public MPU9250Gyro(Port port) {
		pidSourceType = PIDSourceType.kDisplacement;
		pastTime = 0;
		yaw = 0;
		gz = 0;
		gy = 0;
		gx = 0;
		ax = 0;
		ay = 0;
		az = 0;
		mx = 0;
		my = 0;
		mz = 0;
		mRes = 0;
		gScale = GFS_250DPS;
		aScale = AFS_2G;
		mScale = MFS_14BITS;
		if (ADO == 1) {
			MPU9250_ADDRESS = 0x69; // Device address when ADO = 1

		} else {
			MPU9250_ADDRESS = 0x68; // Device address when ADO = 0

		}
		
		i2c = new I2C(port, MPU9250_ADDRESS);
		magI2C = new I2C(port, AK8963_ADDRESS);
		if (i2c.addressOnly()) {
			SmartDashboard.putNumberArray("Gyro Self Test Results", gyroSelfTest());

			SmartDashboard.putString("GYRO", "FOUND");
			
			getGyroRes(gScale);
			getAccelRes(aScale);
			calibrate();
			init();
			if(magI2C.addressOnly()) {
				SmartDashboard.putString("Mag", "FOUND");
				initMagnetometer(magCalibration);
			}
			reset();

		} else {
			SmartDashboard.putString("GYRO", "NOTFOUND");
		}

	}

	/***
	 * Tests the MPU
	 * 
	 */

	 double[] gyroSelfTest() {
		 byte[] rawData = {0, 0, 0, 0, 0, 0};
		 int[] selfTest = new int[6];

		 int[] gAvg = {0, 0, 0}, aAvg = {0, 0, 0}, aSTAvg = {0, 0, 0}, gSTAvg = {0,0, 0};
		 float[] factoryTrim = new float[6];
		 int FS = 0;

		 writeByte(SMPLRT_DIV, 0x00); // Set gyro sample rate to 1kHz
		 writeByte(CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
		 writeByte(GYRO_CONFIG, FS << 3); //Set full scale range for the gyro to 250 dps
		 writeByte(ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
		 writeByte(ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2g

		 for(int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer
			
			readBytes(ACCEL_XOUT_H, 6, rawData); //Read the six raw data registers into data array
			aAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]); //Turn the MSB and LSB into a integer
			aAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]); 
			aAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]);
			
			readBytes(GYRO_XOUT_H, 6, rawData); //Read the six raw data registers into data array
			gAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]); //Turn the MSB and LSB into a integer
			gAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]); 
			gAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]);
			
			

		 }

		 for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
			aAvg[ii] /= 200;
			gAvg[ii] /= 200;

		 }

		 // Configure the accelerometer for self-test
		 writeByte(ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2g
		 writeByte(GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/250 degrees/s
		 
		 Timer.delay(0.025);

		 for (int ii = 0; ii < 200; ii++) { // Get average of 200 values an store as average current readings

			readBytes(ACCEL_XOUT_H, 6, rawData); //Read the six raw data registers into data array
			aSTAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]);
			aSTAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]); 
			aSTAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]);

			readBytes(GYRO_XOUT_H, 6, rawData); //Read the six raw data registers into data array
			gSTAvg[0] += (int)(((int)rawData[0] << 8) | rawData[1]);
			gSTAvg[1] += (int)(((int)rawData[2] << 8) | rawData[3]); 
			gSTAvg[2] += (int)(((int)rawData[4] << 8) | rawData[5]);

		 }

		 for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
			aSTAvg[ii] /= 200;
			gSTAvg[ii] /= 200;

		 }

		// Configure the gyro and accelerometer for normal operation
		writeByte(ACCEL_CONFIG, 0x00);
		writeByte(GYRO_CONFIG, 0x00);

		Timer.delay(0.025);
		
		//Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
		//SELF Test accel results
		selfTest[0] = readByte(SELF_TEST_X_ACCEL); 
		selfTest[1] = readByte(SELF_TEST_Y_ACCEL);
		selfTest[2] = readByte(SELF_TEST_Z_ACCEL);
		//SELF TEST GYRO results
		selfTest[3] = readByte(SELF_TEST_X_GYRO);
		selfTest[4] = readByte(SELF_TEST_Y_GYRO);
		selfTest[5] = readByte(SELF_TEST_Z_GYRO);

		// Retrieve factory self-test value from self-test code reads
		factoryTrim[0] = (float)(2620 / 1 << FS) * (float)(Math.pow(1.01, ((float)selfTest[0] - 1.0) ));
		factoryTrim[1] = (float)(2620 / 1 << FS) * (float)(Math.pow(1.01, ((float)selfTest[1] - 1.0) ));
		factoryTrim[2] = (float)(2620 / 1 << FS) * (float)(Math.pow(1.01, ((float)selfTest[2] - 1.0) ));
		factoryTrim[3] = (float)(2620 / 1 << FS) * (float)(Math.pow(1.01, ((float)selfTest[3] - 1.0) ));
		factoryTrim[4] = (float)(2620 / 1 << FS) * (float)(Math.pow(1.01, ((float)selfTest[4] - 1.0) ));
		factoryTrim[5] = (float)(2620 / 1 << FS) * (float)(Math.pow(1.01, ((float)selfTest[5] - 1.0) ));

		// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
		// To get percent, must multiply by 100
		double[] results = new double[6];
		for (int i = 0; i < 3; i++) {
			results[i]   = (double) (100.0 * ((double)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.); // Report percent differences
			results[i + 3] = (double) (100.0 * ((double)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.); // Report percent differences
		}
		return results;
	}
	/***
	 * Reads an array of bytes
	 * 
	 * @param registerAddress
	 *            Takes in register address
	 * @param count
	 *            Number of bytes of data
	 * @param destination
	 *            Place to store the bytes
	 * @return Returns read bytes
	 */
	byte[] readBytes(int registerAddress, int count, byte[] destination) {
		i2c.read(registerAddress, count, destination);
		return destination;

	}

	byte[] readMagBytes(int registerAddress, int count, byte[] destination) {
		magI2C.read(registerAddress, count, destination);
		return destination;

	}
	/***
	 * Reads only one byte
	 * 
	 * @param registerAddress
	 *            Takes in register address
	 * @return The read byte
	 */
	byte readByte(int registerAddress) {
		byte[] destination = new byte[1];
		i2c.read(registerAddress, 1, destination);
		return destination[0];

	}

	byte readMagByte(int registerAddress) {
		byte[] destination = new byte[1];
		magI2C.read(registerAddress, 1, destination);
		return destination[0];

	}

	/**
	 * Writes data to register
	 * 
	 * @param registerAddress
	 *            Takes in register address
	 * @param data
	 *            Data to write to register
	 * @return True or False based on success of data transfer
	 */
	boolean writeByte(int registerAddress, int data) {
		return i2c.write(registerAddress, data);
	}

	boolean writeByteToMag(int registerAddress, int data) {
		return magI2C.write(registerAddress, data);
	}


	/**
	 * Sets the Gyro Resolution
	 * 
	 * @param gScale
	 *            Gyro Scale
	 */
	void getGyroRes(int gScale) {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
		// value:
		switch (gScale) {
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

	/**
	 * Sets the Accel Resolution
	 * f
	 * @param gScale
	 *            Gyro Scale
	 */
	void getAccelRes(int aScale) {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
		// value:
		switch (aScale) {
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

	/**
	 * Sets the Gyro Resolution
	 * f
	 * @param gScale
	 *            Gyro Scale
	 */
	void getMagRes(int mScale) {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
		// value:
		switch (mScale) {
		case MFS_14BITS:
			mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
			break;
		case MFS_16BITS:
			mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
			break;
		}
	}
	/**
	 * Reads Gyro Data
	 * 
	 * @param destination
	 *            Array to read into
	 */
	void readGyroData(int[] destination) {
		SmartDashboard.putBoolean("Raw Boolean", true);

		byte[] rawData = new byte[6]; //// x/y/z gyro register data stored here
		readBytes(GYRO_XOUT_H, 6, rawData); // Read the six raw data registers sequentially into data array
		double[] dashboardData = new double[rawData.length];
		for (int i = 0; i < rawData.length; i++) {
			dashboardData[i] = (double) rawData[i];
		}
		// System.out.println("This is raw data: " + rawData);
		// SmartDashboard.putNumberArray("Raw Data", dashboardData);
		destination[0] = ((rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		destination[1] = ((rawData[2] << 8) | rawData[3]);
		destination[2] = ((rawData[4] << 8) | rawData[5]);

	}

	/**
	 * Reads Accelerometer Data
	 * 
	 * @param destination
	 *            Array to read into
	 */
	void readAccelData(int[] destination) {
		SmartDashboard.putBoolean("Raw Boolean", true);

		byte[] rawData = new byte[6]; //// x/y/z gyro register data stored here
		readBytes(ACCEL_XOUT_H, 6, rawData); // Read the six raw data registers sequentially into data array
		double[] dashboardData = new double[rawData.length];
		for (int i = 0; i < rawData.length; i++) {
			dashboardData[i] = (double) rawData[i];
		}
		// System.out.println("This is raw data: " + rawData);
		// SmartDashboard.putNumberArray("Raw Data", dashboardData);
		destination[0] = ((rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		destination[1] = ((rawData[2] << 8) | rawData[3]);
		destination[2] = ((rawData[4] << 8) | rawData[5]);

	}

	/**
	 * Reads Magnometer Data
	 * 
	 * @param destination
	 *            Array to read into
	 */
	void readMagData(int[] destination) {
		SmartDashboard.putBoolean("Raw Boolean", true);


		byte[] rawData = new byte[7]; //// x/y/z gyro register data stored here
		
		if((readMagByte(AK8963_ST1) & 0x01) == 1) {
			readMagBytes(AK8963_XOUT_L, 7, rawData);
			int c = rawData[6];
			if((c & 0x08) != 1) {
				destination[0] = ((int)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
				destination[1] = ((int)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
				destination[2] = ((int)rawData[5] << 8) | rawData[4] ;
			}
		}

	}

	int readTempData() {
		byte rawData[] = new byte[2];  // x/y/z gyro register data stored here
	  	readBytes(TEMP_OUT_H, 2, rawData);  // Read the two raw data registers sequentially into data array
	  	return ((int)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
	}

	void initMagnetometer(double[] destination) {
		byte rawData[] = new byte[3];
		writeByteToMag(AK8963_CNTL, 0x00);
		Timer.delay(0.01);
		writeByteToMag(AK8963_CNTL, 0x0F);
		Timer.delay(0.01);
		readMagBytes(AK8963_ASAX, 3, rawData);

		destination[0] =  (double)((rawData[0] - 128) / 256. + 1.); // Return x-axis sensitivity adjustment values, etc.
		destination[1] =  (double)((rawData[1] - 128) / 256. + 1.);
		destination[2] =  (double)((rawData[2] - 128) / 256. + 1.);

		writeByteToMag(AK8963_CNTL, 0x00); // Power down 
		Timer.delay(0.01);
		// Configure the magnetometer for continuous read and highest resolution
		// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
		// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
		writeByteToMag(AK8963_CNTL, mScale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
		Timer.delay(.01);

	}


	/**
	 * Initializes Gyro
	 */
	void init() {
		// wake up device
		writeByte(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
		Timer.delay(.100); // Wait for all registers to reset

		// get stable time source
		writeByte(PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
		Timer.delay(.200);

		// Configure Gyro and Thermometer
		// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
		// respectively;
		// minimum delay time for this setting is 5.9 ms, which means sensor fusion
		// update rates cannot
		// be higher than 1 / 0.0059 = 170 Hz
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
		// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8
		// kHz, or 1 kHz
		writeByte(CONFIG, 0x03);

		// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		writeByte(SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
										// determined inset in CONFIG above

		// Set gyroscope full scale range
		// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted
		// into positions 4:3
		int c = readByte(GYRO_CONFIG); // get current GYRO_CONFIG register value
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = (c & ~0x03); // Clear Fchoice bits [1:0]
		c = (c & ~0x18); // Clear GFS bits [4:3]
		c = c | gScale << 3; // Set full scale range for the gyro
		// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits
		// 1:0 of GYRO_CONFIG
		writeByte(GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

		// Set accelerometer full-scale range configuration
		c = readByte(ACCEL_CONFIG); // get current ACCEL_CONFIG register value
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x18;  // Clear AFS bits [4:3]
		c = c | aScale << 3; // Set full scale range for the accelerometer
		writeByte(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

		// Set accelerometer sample rate configuration
		// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
		// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
		c = readByte( ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
		c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
		c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
		writeByte(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
		// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
		// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting


		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until
		// interrupt cleared,
		// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
		// can join the I2C bus and all can be controlled by the Arduino as master
		// writeByte(INT_PIN_CFG, 0x22);
		writeByte(INT_PIN_CFG, 0x12); // INT is 50 microsecond pulse and any read to clear
		writeByte(INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
		Timer.delay(.100);

	}

	@Override
	public void calibrate() {

		calibrateMPU9250(gyroBias, accelBias);
	}

	void calibrateMPU9250(double[] dest1, double[] dest2) {
		byte data[] = new byte[12]; // data array to hold accelerometer and gyro x, y, z, data
		int ii, packet_count, fifo_count;
		int gyro_bias[] = { 0, 0, 0 }, accel_bias[] = {0,0,0};


		// reset device, reset all registers, clear gyro and accelerometer bias
		// registers
		writeByte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
		Timer.delay(0.1);

		// get stable time source
		// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
		writeByte(PWR_MGMT_1, 0x01);
		writeByte(PWR_MGMT_2, 0x00);
		Timer.delay(0.2);

		// Configure device for bias calculation
		writeByte(INT_ENABLE, 0x00); // Disable all interrupts
		writeByte(FIFO_EN, 0x00); // Disable FIFO
		writeByte(PWR_MGMT_1, 0x00); // Turn on internal clock source
		writeByte(I2C_MST_CTRL, 0x00); // Disable I2C master
		writeByte(USER_CTRL, 0x00); // Disable FIFO and I2C master modes
		writeByte(USER_CTRL, 0x0C); // Reset FIFO and DMP
		Timer.delay(0.015);

		// Configure MPU9250 gyro and accelerometer for bias calculation
		writeByte(CONFIG, 0x01); // Set low-pass filter to 188 Hz
		writeByte(SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
		writeByte(GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
		writeByte(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2g, maximum sensitivity



		int gyrosensitivity = 131; // = 131 LSB/degrees/sec
		int accelsensitivity = 16384; // 16384 LSB/g



		// Configure FIFO to capture accelerometer and gyro data for bias calculation
		writeByte(USER_CTRL, 0x40); // Enable FIFO
		writeByte(FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in
									// MPU-9250)
		Timer.delay(0.04); // accumulate 40 samples in 80 milliseconds = 480 bytes

		// At end of sample accumulation, turn off FIFO sensor read
		writeByte(FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
		readBytes(FIFO_COUNTH, 2, data); // read FIFO sample count
		fifo_count = (data[0] << 8) | data[1];
		packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

		for (ii = 0; ii < packet_count; ii++) {
			int gyro_temp[] = { 0, 0, 0 }, accel_temp[] = {0, 0, 0};
			readBytes(FIFO_R_W, 12, data); // read data for averaging

			accel_temp[0] = ((data[0] << 8) | data[1]);
			accel_temp[1] = ((data[2] << 8) | data[3]);
			accel_temp[2] = ((data[4] << 8) | data[5]);
			gyro_temp[0] = ((data[6] << 8) | data[7]);
			gyro_temp[1] = ((data[8] << 8) | data[9]);
			gyro_temp[2] = ((data[10] << 8) | data[11]);

			// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases

			accel_bias[0] += accel_temp[0];
			accel_bias[1] += accel_temp[1];
			accel_bias[2] += accel_temp[2];
			gyro_bias[0] += gyro_temp[0];
			gyro_bias[1] += gyro_temp[1];
			gyro_bias[2] += gyro_temp[2];

		}
		// Normalize sums to get average count biases
		// TODO Divides by zero
		try {
			accel_bias[0] /= packet_count;
			accel_bias[1] /= packet_count;
			accel_bias[2] /= packet_count;
			gyro_bias[0] /= packet_count;
			gyro_bias[1] /= packet_count;
			gyro_bias[2] /= packet_count;
		} catch (Exception e) {
			SmartDashboard.putString("GYRO_STATUS", "ZERO");
			for (int i = 0; i < 100; i++)
				System.out.println("Packet count = 0; fifo count = 0");
		}

		if (accel_bias[2] > 0L) {
			accel_bias[2] -= (int) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
		} else {
			accel_bias[2] += (int) accelsensitivity;
		}
		// Construct the gyro biases for push to the hardware gyro bias registers, which
		// are reset to zero upon device startup
		data[0] = (byte) ((-gyro_bias[0] / 4 >> 8) & 0xFF); // Divide by 4 to get 32.9 LSB per deg/s to conform to
															// expected bias input format
		data[1] = (byte) ((-gyro_bias[0] / 4) & 0xFF); // Biases are additive, so change sign on calculated average gyro
														// biases
		data[2] = (byte) ((-gyro_bias[1] / 4 >> 8) & 0xFF);
		data[3] = (byte) ((-gyro_bias[1] / 4) & 0xFF);
		data[4] = (byte) ((-gyro_bias[2] / 4 >> 8) & 0xFF);
		data[5] = (byte) ((-gyro_bias[2] / 4) & 0xFF);

		/// Push gyro biases to hardware registers
		/*
		 * writeByte(XG_OFFSET_H, data[0]); writeByte(XG_OFFSET_L, data[1]);
		 * writeByte(YG_OFFSET_H, data[2]); writeByte(YG_OFFSET_L, data[3]);
		 * writeByte(ZG_OFFSET_H, data[4]); writeByte(ZG_OFFSET_L, data[5]);
		 */
		writeByte(XG_OFFSET_H, data[0]);
		writeByte(XG_OFFSET_L, data[1]);
		writeByte(YG_OFFSET_H, data[2]);
		writeByte(YG_OFFSET_L, data[3]);
		writeByte(ZG_OFFSET_H, data[4]);
		writeByte(ZG_OFFSET_L, data[5]);

		dest1[0] = gyro_bias[0] / gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
		dest1[1] = gyro_bias[1] / gyrosensitivity;
		dest1[2] = gyro_bias[2] / gyrosensitivity;

		 // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
		// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
		// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
		// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
		// the accelerometer biases calculated above must be divided by 8.

		int accel_bias_reg[] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
		readBytes(XA_OFFSET_H, 2, data);
		accel_bias_reg[0] = (int) (((int)data[0] << 8) | data[1]);
		readBytes(YA_OFFSET_H, 2, data);
		accel_bias_reg[1] = (int) (((int)data[0] << 8) | data[1]);
		readBytes(ZA_OFFSET_H, 2, data);
		accel_bias_reg[2] = (int) (((int)data[0] << 8) | data[1]);

		int mask = 1; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
		int mask_bit[] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

		for(ii = 0; ii < 3; ii++) {
			if((accel_bias_reg[ii] & mask) == 1) 
				mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
		}

		// Construct total accelerometer bias, including calculated average accelerometer bias from above
		accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
		accel_bias_reg[1] -= (accel_bias[1] / 8);
		accel_bias_reg[2] -= (accel_bias[2] / 8);

		data[0] = (byte) ((accel_bias_reg[0] >> 8) & 0xFF);
		data[1] = (byte) ((accel_bias_reg[0])      & 0xFF);
		data[1] = (byte) (data[1] | mask_bit[0]); // preserve temperature compensation bit when writing back to accelerometer bias registers
		data[2] = (byte) ((accel_bias_reg[1] >> 8) & 0xFF);
		data[3] = (byte) ((accel_bias_reg[1])      & 0xFF);
		data[3] = (byte) (data[3] | mask_bit[1]); // preserve temperature compensation bit when writing back to accelerometer bias registers
		data[4] = (byte) ((accel_bias_reg[2] >> 8) & 0xFF);
		data[5] = (byte) ((accel_bias_reg[2])      & 0xFF);
		data[5] = (byte) (data[5] | mask_bit[2]); // preserve temperature compensation bit when writing back to accelerometer bias registers
	  
		// Apparently this is not working for the acceleration biases in the MPU-9250
		// Are we handling the temperature correction bit properly?
		// Push accelerometer biases to hardware registers
		writeByte(XA_OFFSET_H, data[0]);
		writeByte(XA_OFFSET_L, data[1]);
		writeByte(YA_OFFSET_H, data[2]);
		writeByte(YA_OFFSET_L, data[3]);
		writeByte(ZA_OFFSET_H, data[4]);
		writeByte(ZA_OFFSET_L, data[5]);
	  
		// Output scaled accelerometer biases for display in the main program
		dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
		dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
		dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
	}

	@Override
	public double getAngle() {
		return yaw - offset;
	}

	public double getGyroTemperature() {
		return temperature;
	}



	/*** Angular Acceleration ***/
	@Override
	public double getRate() {
		// TODO Auto-generated method stub
		return gz;
	}

	@Override
	public void reset() {

		writeByte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
		yaw = 0;
		Timer.delay(0.1);
	}

	// Updates in loop
	public void update() {
		// Multiply timer.getTimestamp * 1000 to get millis
		double currentTimestamp = Timer.getFPGATimestamp() * 1000;
		readAccelData(accelCount);
		getAccelRes(aScale);


		// Now we'll calculate the accleration value into actual g's
		ax = (double)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
		ay = (double)accelCount[1] * aRes; // - accelBias[1];
		az = (double)accelCount[2] * aRes; // - accelBias[2];


		// TODO change to account for drift
		double rotation_threshold = .5;
		SmartDashboard.putBoolean("RUN UPDATE", true);
		readGyroData(gyroData);
		getGyroRes(gScale);
		// I think gz is rate of change
		gx = gyroData[0] * gRes;
		gy = gyroData[1] * gRes;
		gz = gyroData[2] * gRes;

		readMagData(magCount);
		getMagRes(mScale);

		magBias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
		magBias[1] = +120.;  // User environmental x-axis correction in milliGauss
		magBias[2] = +125.;  // User environmental x-axis correction in milliGauss
			

		mx = (double)magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
    	my = (double)magCount[1] * mRes * magCalibration[1] - magBias[1];
		mz = (double)magCount[2] * mRes * magCalibration[2] - magBias[2];
		
		currentTime = Timer.getFPGATimestamp();
		deltat = currentTime - pastTime;
		pastTime = currentTime;

		sum += deltat;
		sumCount++;
		MahonyQuaternionUpdate(ax, ay, az, gx * Math.PI / 180, gy * Math.PI / 180.0f, gz * Math.PI / 180, my, mx, mz);
		 // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
		// In this coordinate system, the positive z-axis is down toward Earth.
		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
		yaw   = Math.atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		pitch = -Math.asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll  = Math.atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		pitch *= 180.0f / Math.PI;
		yaw   *= 180.0f / Math.PI;
		yaw   -= 0; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		roll  *= 180.0f / Math.PI;

		tempCount = readTempData();  // Read the adc values
        temperature = ((double) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
	}


	void MahonyQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz)
		{
		double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
		double norm;
		double hx, hy, bx, bz;
		double vx, vy, vz, wx, wy, wz;
		double ex, ey, ez;
		double pa, pb, pc;

		// Auxiliary variables to avoid repeated arithmetic
		double q1q1 = q1 * q1;
		double q1q2 = q1 * q2;
		double q1q3 = q1 * q3;
		double q1q4 = q1 * q4;
		double q2q2 = q2 * q2;
		double q2q3 = q2 * q3;
		double q2q4 = q2 * q4;
		double q3q3 = q3 * q3;
		double q3q4 = q3 * q4;
		double q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = Math.sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = Math.sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
		hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
		bx = Math.sqrt((hx * hx) + (hy * hy));
		bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

		// Estimated direction of gravity and magnetic field
		vx = 2.0f * (q2q4 - q1q3);
		vy = 2.0f * (q1q2 + q3q4);
		vz = q1q1 - q2q2 - q3q3 + q4q4;
		wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
		wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
		wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

		// Error is cross product between estimated direction and measured direction of gravity
		ex = (ay * vz - az * vy) + (my * wz - mz * wy);
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
		if (Ki > 0.0f)
		{
			eInt[0] += ex;      // accumulate integral error
			eInt[1] += ey;
			eInt[2] += ez;
		}
		else
		{
			eInt[0] = 0.0f;     // prevent integral wind up
			eInt[1] = 0.0f;
			eInt[2] = 0.0f;
		}

		// Apply feedback terms
		gx = gx + Kp * ex + Ki * eInt[0];
		gy = gy + Kp * ey + Ki * eInt[1];
		gz = gz + Kp * ez + Ki * eInt[2];

		// Integrate rate of change of quaternion
		pa = q2;
		pb = q3;
		pc = q4;
		q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
		q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
		q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
		q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

		// Normalise quaternion
		norm = Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
		norm = 1.0f / norm;
		q[0] = q1 * norm;
		q[1] = q2 * norm;
		q[2] = q3 * norm;
		q[3] = q4 * norm;

}
	@Override
	public void free() {
		// TODO Auto-generated method stub
		i2c.free();
	}

	/***********************************************************/
	/* PIDSource Interface Implementation */
	/***********************************************************/

	public PIDSourceType getPIDSourceType() {
		return pidSourceType;
	}

	public void setPIDSourceType(PIDSourceType type) {
		pidSourceType = type;
	}

	/**
	 * Returns the current yaw value reported by the sensor. This yaw value is
	 * useful for implementing features including "auto rotate to a known angle".
	 * 
	 * @return The current yaw angle in degrees (-180 to 180).
	 */
	public double pidGet() {

		return getAngle();

	}

	public double sampleGyroData(double gz) {
		double sampledGyroData = 0;
		// Shift samples
		for (int i = 0; i < (NUM_SAMPLES - 1); i++) {
			gyroSamples[i] = gyroSamples[i + 1];
		}
		// Set current sample to last cell
		gyroSamples[NUM_SAMPLES - 1] = gz;
		// Average samples
		for (int i = 0; i < NUM_SAMPLES; i++) {
			sampledGyroData += gyroSamples[i];
		}
		sampledGyroData /= NUM_SAMPLES;

		return sampledGyroData;

	}

	public void setOffset() {
		offset = getAngle();
	}

}
