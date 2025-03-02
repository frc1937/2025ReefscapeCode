package frc.lib.generic.hardware.encoder.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.hardware.HardwareManager;
import frc.lib.generic.hardware.encoder.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

import static frc.lib.generic.hardware.encoder.EncoderInputs.ENCODER_INPUTS_LENGTH;
import static frc.lib.util.QueueUtilities.queueToDoubleArray;

/**
 * Wrapper class for the CAN encoder.
 * Verify its setup is correct via this:
 * <a href="https://store.ctr-electronics.com/content/user-manual/CANCoder%20User">CTRE CANcoder PDF</a>'s%20Guide.pdf
 */
public class GenericCanCoder extends Encoder {
    private final boolean[] signalsToLog = new boolean[ENCODER_INPUTS_LENGTH];

    private final CANcoder canCoder;
    private final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;

    public GenericCanCoder(String name, int canCoderID, String canbusName) {
        super(name);

        canCoder = new CANcoder(canCoderID, canbusName);

        positionSignal = canCoder.getPosition().clone();
        velocitySignal = canCoder.getVelocity().clone();
    }

    public GenericCanCoder(String name, int canCoderID) {
        this(name, canCoderID, "");
    }

    @Override
    public void setupSignalUpdates(EncoderSignal signal, boolean useFasterThread) {
        signalsToLog[signal.getId()] = true;

        if (!useFasterThread) {
            switch (signal) {
                case POSITION -> setupNonThreadedSignal(positionSignal);
                case VELOCITY -> setupNonThreadedSignal(velocitySignal);
            }

            return;
        }

        signalsToLog[signal.getId() + ENCODER_INPUTS_LENGTH / 2] = true;

        switch (signal) {
            case POSITION -> setupThreadedSignal("position", positionSignal);
            case VELOCITY -> setupThreadedSignal("velocity", velocitySignal);
        }
    }

    @Override
    public boolean configure(EncoderConfiguration encoderConfiguration) {
        canCoderConfig.MagnetSensor.MagnetOffset = encoderConfiguration.offsetRotations;

        canCoderConfig.MagnetSensor.SensorDirection = encoderConfiguration.invert ?
                SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
                encoderConfiguration.sensorRange == EncoderProperties.SensorRange.ZERO_TO_ONE ? 1 : 0.5;

        canCoder.optimizeBusUtilization();

        return applyConfig();
    }

    private boolean applyConfig() {
        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = canCoder.getConfigurator().apply(canCoderConfig);
            counter--;
        }

        return statusCode == StatusCode.OK;
    }

    @Override
    protected boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    protected void refreshInputs(EncoderInputs inputs) {
        if (canCoder == null) return;

        inputs.setSignalsToLog(signalsToLog);

        inputs.position = positionSignal.getValueAsDouble();
        inputs.velocity = velocitySignal.getValueAsDouble();

        if (signalQueueList.isEmpty()) return;

        inputs.threadPosition = queueToDoubleArray(signalQueueList.get("position"));
        inputs.threadVelocity = queueToDoubleArray(signalQueueList.get("velocity"));
    }

    private void setupNonThreadedSignal(final BaseStatusSignal correspondingSignal) {
        correspondingSignal.setUpdateFrequency(50);
        HardwareManager.registerCTREStatusSignal(correspondingSignal);
    }

    private void setupThreadedSignal(String name, BaseStatusSignal signal) {
        signal.setUpdateFrequency(200);
        signalQueueList.put(name, OdometryThread.getInstance().registerCTRESignal(signal));
    }
}
