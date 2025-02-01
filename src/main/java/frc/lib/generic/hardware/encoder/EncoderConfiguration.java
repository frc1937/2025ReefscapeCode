package frc.lib.generic.hardware.encoder;

public class EncoderConfiguration { 
    /** The offset to zero the cancoder, in rotations <p> This gets added to all reported encoder positions </p> */
    public double offsetRotations = 0;

    /** False -> direction is CCW+ CW-. True -> direction is CW+ CCW- */
    public boolean invert = false;

    /**
     * Set the range of the encoder data.
     * <p> Verify sensor direction
     * <a href="https://pro.docs.ctr-electronics.com/en/latest/docs/hardware-reference/cancoder/index.html#verifying-sensor-direction">CTRE CANcoder documentation</a>
     * */
    public EncoderProperties.SensorRange sensorRange = EncoderProperties.SensorRange.ZERO_TO_ONE;
}
