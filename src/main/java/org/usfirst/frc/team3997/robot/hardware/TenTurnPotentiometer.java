package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.hal.AnalogJNI;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TenTurnPotentiometer{
	
	  int port; // explicit no modifier, private and package accessible.
	  private int channel;
	  private static final int[] kAccumulatorChannels = {0, 1};
	  //Gear 4 to 1 ratio
	  private static final int VOLT_TO_DEGREES = 720/4;
	  double starting_error;
	  protected PIDSourceType m_pidSource = PIDSourceType.kDisplacement;

	  /**
	   * Construct an analog channel.
	   *
	   * @param channel The channel number to represent. 0-3 are on-board 4-7 are on the MXP port.
	   */
	   public TenTurnPotentiometer(final int channel) {
		    this.channel = channel;
		    final int portHandle = AnalogJNI.getPort((byte) channel);
		    port = AnalogJNI.initializeAnalogInputPort(portHandle);
		    //temp
		    starting_error = (getAverageVoltage() * VOLT_TO_DEGREES);
		    HAL.report(tResourceType.kResourceType_AnalogChannel, this.channel);
	   }
	   /**
		   * Channel destructor.
		   */
		  public void free() {
		    AnalogJNI.freeAnalogInputPort(port);
		    port = 0;
		    channel = 0;
		  }

		  /**
		   * Get a sample straight from this channel. The sample is a 12-bit value representing the 0V to 5V
		   * range of the A/D converter. The units are in A/D converter codes. Use GetVoltage() to get the
		   * analog value in calibrated units.
		   *
		   * @return A sample straight from this channel.
		   */
		  public int getValue() {
		    return AnalogJNI.getAnalogValue(port);
		  }

		  /**
		   * Get a sample from the output of the oversample and average engine for this channel. The sample
		   * is 12-bit + the bits configured in SetOversampleBits(). The value configured in
		   * setAverageBits() will cause this value to be averaged 2^bits number of samples. This is not a
		   * sliding window. The sample will not change until 2^(OversampleBits + AverageBits) samples have
		   * been acquired from this channel. Use getAverageVoltage() to get the analog value in calibrated
		   * units.
		   *
		   * @return A sample from the oversample and average engine for this channel.
		   */
		  public int getAverageValue() {
		    return AnalogJNI.getAnalogAverageValue(port);
		  }

		  /**
		   * Get a scaled sample straight from this channel. The value is scaled to units of Volts using the
		   * calibrated scaling data from getLSBWeight() and getOffset().
		   *
		   * @return A scaled sample straight from this channel.
		   */
		  public double getVoltage() {
			  //return 0;
		      return AnalogJNI.getAnalogVoltage(port);
		  }

		  /**
		   * Get a scaled sample from the output of the oversample and average engine for this channel. The
		   * value is scaled to units of Volts using the calibrated scaling data from getLSBWeight() and
		   * getOffset(). Using oversampling will cause this value to be higher resolution, but it will
		   * update more slowly. Using averaging will cause this value to be more stable, but it will update
		   * more slowly.
		   *
		   * @return A scaled sample from the output of the oversample and average engine for this channel.
		   */
		  public double getAverageVoltage() {
			  return AnalogJNI.getAnalogAverageVoltage(port);
		  }


		  /**
		   * Set the sample rate per channel.
		   *
		   * <p>This is a global setting for all channels. The maximum rate is 500kS/s divided by the number
		   * of channels in use. This is 62500 samples/s per channel if all 8 channels are used.
		   *
		   * @param samplesPerSecond The number of samples per second.
		   */
		  public static void setGlobalSampleRate(final double samplesPerSecond) {
		    AnalogJNI.setAnalogSampleRate(samplesPerSecond);
		  }

		  /**
		   * Get the current sample rate.
		   *
		   * <p>This assumes one entry in the scan list. This is a global setting for all channels.
		   *
		   * @return Sample rate.
		   */
		  public static double getGlobalSampleRate() {
		    return AnalogJNI.getAnalogSampleRate();
		  }

		  public void setPIDSourceType(PIDSourceType pidSource) {
		    m_pidSource = pidSource;
		  }

		  public PIDSourceType getPIDSourceType() {
		    return m_pidSource;
		  }

		  /**
		   * Get the average voltage for use with PIDController.
		   *
		   * @return the average voltage
		   */
		  public double pidGet() {
		    return getAverageVoltage();
		  }

		  public void initSendable(SendableBuilder builder) {
		    builder.setSmartDashboardType("Analog Input");
		    builder.addDoubleProperty("Value", this::getAverageVoltage, null);
			}
			/**Returns the current angle of the arm in degrees */
		  public double getAngle() {
			  SmartDashboard.putNumber("AVERAGE_POT_VolTAGE", getAverageVoltage());
				//Sets the offset of the pot during disabled
				if(RobotState.isDisabled())
					starting_error = (getAverageVoltage() * VOLT_TO_DEGREES);
				//Gets the current angle - offset
				double degrees = (getAverageVoltage() * VOLT_TO_DEGREES) - starting_error;
				return degrees;
		  }
}
