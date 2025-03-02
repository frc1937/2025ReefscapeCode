package frc.lib.generic.hardware.sensors;

import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.HardwareManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Sensor implements LoggableHardware {
    private final SensorInputsAutoLogged inputs = new SensorInputsAutoLogged();
    private final String name;

    public Sensor(String name) {
        this.name = "Sensors/" +name;

        periodic();
        HardwareManager.addHardware(this);
    }

    public int get() {
        return inputs.currentValue;
    }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public SensorInputsAutoLogged getInputs() {
        return inputs;
    }

    public void refreshInputs(SensorInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class SensorInputs {
        public int currentValue = 0;
        public int[] threadCurrentValue = new int[0];
    }
}
