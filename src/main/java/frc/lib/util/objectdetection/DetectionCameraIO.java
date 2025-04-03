package frc.lib.util.objectdetection;

import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.HardwareManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class DetectionCameraIO implements LoggableHardware {
    private final String name;
    private final DetectionCameraInputsAutoLogged inputs = new DetectionCameraInputsAutoLogged();

    public DetectionCameraIO(String name) {
        this.name = name;

        periodic();
        HardwareManager.addHardware(this);
    }

    protected void refreshInputs(DetectionCameraInputsAutoLogged inputs) { }

    public boolean hasResult() {
        return inputs.yaws != null && inputs.yaws.length > 0;
    }

    public double getYawToClosestTarget() {
        return inputs.closestTargetYaw;
    }

    public double getPitchToClosestTarget() {
        return inputs.closestTargetPitch;
    }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs("ObjectCameras/" + name, inputs);
    }

    @Override
    public DetectionCameraInputsAutoLogged getInputs() {
        return inputs;
    }

    @AutoLog
    public static class DetectionCameraInputs {
        public double closestTargetYaw;
        public double closestTargetPitch;
        public double[] yaws;
    }
}
