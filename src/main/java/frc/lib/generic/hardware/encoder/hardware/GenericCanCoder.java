package frc.lib.generic.hardware.encoder.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.generic.hardware.signals.FasterSignalsThread;
import frc.lib.generic.hardware.encoder.*;

import java.util.*;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.lib.generic.hardware.encoder.EncoderInputs.ENCODER_INPUTS_LENGTH;

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

    private final List<BaseStatusSignal> signalsToUpdateList = new ArrayList<>();
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
        final int updateFrequency = useFasterThread ? 200 : 50;

        signalsToLog[signal.getId()] = true;

        switch (signal) {
            case POSITION -> setupSignal(positionSignal, updateFrequency);
            case VELOCITY -> setupSignal(velocitySignal, updateFrequency);
        }

        if (!useFasterThread) return;

        signalsToLog[signal.getId() + ENCODER_INPUTS_LENGTH / 2] = true;

        switch (signal) {
            case POSITION -> signalQueueList.put("position", FasterSignalsThread.getInstance().registerSignal(this::getEncoderPositionPrivate));
            case VELOCITY -> signalQueueList.put("velocity", FasterSignalsThread.getInstance().registerSignal(this::getEncoderVelocityPrivate));
        }
    }

    @Override
    public boolean configure(EncoderConfiguration encoderConfiguration) {
        canCoderConfig.MagnetSensor.MagnetOffset = encoderConfiguration.offsetRotations;

        canCoderConfig.MagnetSensor.SensorDirection = encoderConfiguration.invert ?
                SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
                encoderConfiguration.sensorRange == EncoderProperties.SensorRange.ZERO_TO_ONE

                ? 1 : 0.5;

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

        BaseStatusSignal.refreshAll(signalsToUpdateList.toArray(new BaseStatusSignal[0]));

        inputs.position = getEncoderPositionPrivate();
        inputs.velocity = getEncoderVelocityPrivate();

        if (signalQueueList.isEmpty()) return;

        if (signalQueueList.get("position") != null)
            inputs.threadPosition = signalQueueList.get("position").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("velocity") != null)
            inputs.threadVelocity = signalQueueList.get("velocity").stream().mapToDouble(Double::doubleValue).toArray();

        signalQueueList.forEach((k, v) -> v.clear());
    }

    private double getEncoderPositionPrivate() {
        return positionSignal.getValue().in(Rotations);
    }

    private double getEncoderVelocityPrivate() {
        return velocitySignal.getValue().in(RotationsPerSecond);
    }

    private void setupSignal(final BaseStatusSignal correspondingSignal, int updateFrequency) {
        signalsToUpdateList.add(correspondingSignal);
        correspondingSignal.setUpdateFrequency(updateFrequency);
    }
}
