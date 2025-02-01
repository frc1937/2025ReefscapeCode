package frc.lib.generic.hardware.sensors.hardware;

import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorInputsAutoLogged;

public class DigitalInput extends Sensor {
    private final edu.wpi.first.wpilibj.DigitalInput digitalInput;

    public DigitalInput(String name, int id) {
        super(name);

        digitalInput = new edu.wpi.first.wpilibj.DigitalInput(id);
    }

    @Override
    public void refreshInputs(SensorInputsAutoLogged inputs) {
        if (digitalInput == null) return;

        inputs.currentValue = digitalInput.get() ? 1 : 0;

        //todo: add support for faster akit inputs
        // LOL!!! Our intake runs at 100HZ ahhh comment
    }
}
