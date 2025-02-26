package frc.lib.generic.hardware.pigeon.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.hardware.HardwareManager;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonConfiguration;
import frc.lib.generic.hardware.pigeon.PigeonInputs;
import frc.lib.generic.hardware.pigeon.PigeonSignal;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

import static frc.lib.generic.hardware.pigeon.PigeonInputs.PIGEON_INPUTS_LENGTH;
import static frc.lib.generic.hardware.pigeon.hardware.PigeonUtilities.handleThreadedInputs;

public class GenericPigeon2 extends Pigeon {
    private final Pigeon2 pigeon;
    private double yawOffset = 0;

    private final StatusSignal<Angle> yawSignal, pitchSignal, rollSignal;

    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();
    private final boolean[] signalsToLog = new boolean[PIGEON_INPUTS_LENGTH];

    public GenericPigeon2(String name, int deviceNumber) {
        super(name);

        pigeon = new Pigeon2(deviceNumber);

        yawSignal = pigeon.getYaw().clone();
        pitchSignal = pigeon.getPitch().clone();
        rollSignal = pigeon.getRoll().clone();
    }


    @Override
    public void configurePigeon(PigeonConfiguration pigeonConfiguration) {
        pigeon.reset();

        final Pigeon2Configuration configuration  = new Pigeon2Configuration();

        yawOffset = pigeonConfiguration.mountPoseYawDegrees;

        configuration.MountPose.MountPoseYaw = pigeonConfiguration.mountPoseYawDegrees;
        configuration.MountPose.MountPosePitch = pigeonConfiguration.mountPosePitchDegrees;
        configuration.MountPose.MountPoseRoll = pigeonConfiguration.mountPoseRollDegrees;

        pigeon.optimizeBusUtilization();

        pigeon.getConfigurator().apply(configuration.Pigeon2Features.withEnableCompass(false));
    }

    @Override
    public void setGyroYaw(double yawRotations) {
        pigeon.setYaw(yawRotations * 360);
    }

    @Override
    public boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    protected void refreshInputs(PigeonInputs inputs) {
        if (pigeon == null) return;

        inputs.setSignalsToLog(signalsToLog);

        inputs.gyroYawRotations = (yawSignal.getValueAsDouble() - yawOffset) / 360;
        inputs.gyroPitchRotations = pitchSignal.getValueAsDouble() / 360;
        inputs.gyroRollRotations = rollSignal.getValueAsDouble() / 360;

        handleThreadedInputs(inputs, signalQueueList);
    }

    @Override
    public void setupSignalUpdates(PigeonSignal signal, boolean useFasterThread) {
        signalsToLog[signal.getId()] = true;

        if (!useFasterThread) {
            switch (signal) {
                case YAW -> setupNonThreadedSignal(yawSignal);
                case ROLL -> setupNonThreadedSignal(rollSignal);
                case PITCH -> setupNonThreadedSignal(pitchSignal);
            }

            return;
        }

        signalsToLog[signal.getId() + PIGEON_INPUTS_LENGTH / 2] = true;

        switch (signal) {
            case YAW -> setupThreadedSignal("yaw_pigeon2", yawSignal);
            case ROLL -> setupThreadedSignal("roll_pigeon2", rollSignal);
            case PITCH -> setupThreadedSignal("pitch_pigeon2", pitchSignal);
        }
    }


    private void setupNonThreadedSignal(final BaseStatusSignal signal) {
        signal.setUpdateFrequency(50);
        HardwareManager.registerCTREStatusSignal(signal);
    }

    private void setupThreadedSignal(String name, BaseStatusSignal signal) {
        signal.setUpdateFrequency(200);
        signalQueueList.put(name, OdometryThread.getInstance().registerCTRESignal(signal));
    }
}
