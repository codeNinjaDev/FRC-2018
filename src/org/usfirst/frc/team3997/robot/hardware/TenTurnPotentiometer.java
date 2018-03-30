package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.hal.AnalogJNI;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.util.AllocationException;

public class TenTurnPotentiometer extends AnalogPotentiometer{
	
	  private static final int kAccumulatorSlot = 1;
	  int m_port; // explicit no modifier, private and package accessible.
	  private int m_channel;
	  private static final int[] kAccumulatorChannels = {0, 1};
	  //Gear 4 to 1 ratio
	  private static final int VOLT_TO_DEGREES = 720/4;
	  
	  protected PIDSourceType m_pidSource = PIDSourceType.kDisplacement;

	  /**
	   * Construct an analog channel.
	   *
	   * @param channel The channel number to represent. 0-3 are on-board 4-7 are on the MXP port.
	   */
	   public TenTurnPotentiometer(final int channel) {
		    super(channel);
		    checkAnalogInputChannel(channel);
		    m_channel = channel;

		    final int portHandle = AnalogJNI.getPort((byte) channel);
		    //m_port = AnalogJNI.initializeAnalogInputPort(portHandle);
		    //temp
		    m_port = 0;
		    
		    HAL.report(tResourceType.kResourceType_AnalogChannel, channel);
		    setName("AnalogInput", channel);
	   }
	   /**
		   * Channel destructor.
		   */
		  @Override
		  public void free() {
		    super.free();
		    AnalogJNI.freeAnalogInputPort(m_port);
		    m_port = 0;
		    m_channel = 0;
		  }

		  /**
		   * Get a sample straight from this channel. The sample is a 12-bit value representing the 0V to 5V
		   * range of the A/D converter. The units are in A/D converter codes. Use GetVoltage() to get the
		   * analog value in calibrated units.
		   *
		   * @return A sample straight from this channel.
		   */
		  public int getValue() {
		    return AnalogJNI.getAnalogValue(m_port);
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
		    return AnalogJNI.getAnalogAverageValue(m_port);
		  }

		  /**
		   * Get a scaled sample straight from this channel. The value is scaled to units of Volts using the
		   * calibrated scaling data from getLSBWeight() and getOffset().
		   *
		   * @return A scaled sample straight from this channel.
		   */
		  public double getVoltage() {
			  //return 0;
		      return AnalogJNI.getAnalogVoltage(m_port);
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
			  return AnalogJNI.getAnalogAverageVoltage(m_port);
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

		  @Override
		  public void setPIDSourceType(PIDSourceType pidSource) {
		    m_pidSource = pidSource;
		  }

		  @Override
		  public PIDSourceType getPIDSourceType() {
		    return m_pidSource;
		  }

		  /**
		   * Get the average voltage for use with PIDController.
		   *
		   * @return the average voltage
		   */
		  @Override
		  public double pidGet() {
		    return getAverageVoltage();
		  }

		  @Override
		  public void initSendable(SendableBuilder builder) {
		    builder.setSmartDashboardType("Analog Input");
		    builder.addDoubleProperty("Value", this::getAverageVoltage, null);
		  }
		  public double getAngle() {
			double degrees = (getAverageVoltage() * VOLT_TO_DEGREES);
			return degrees;
		  }
}
