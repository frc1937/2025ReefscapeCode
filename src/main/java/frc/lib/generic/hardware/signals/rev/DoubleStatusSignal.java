package frc.lib.generic.hardware.signals.rev;

import frc.lib.generic.hardware.motor.MotorSignal;

import java.util.function.DoubleSupplier;

public class DoubleStatusSignal {
    private final DoubleSupplier valueSupplier;
    private final String name;

    public DoubleStatusSignal(MotorSignal signal, DoubleSupplier valueSupplier) {
        this.name = signal.name();
        this.valueSupplier = valueSupplier;
    }

    public double getValue() {
        return valueSupplier.getAsDouble();
    }

    public DoubleSupplier getValueSupplier() {
        return valueSupplier;
    }

    public String getName() {
        return name;
    }
}
