package frc.lib.generic.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import frc.lib.generic.simulation.extensions.ExtendedDCMotorSim;

import static frc.robot.GlobalConstants.ROBOT_PERIODIC_LOOP_TIME;

public class SimpleMotorSimulation extends GenericPhysicsSimulation {
    private final ExtendedDCMotorSim motorSimulation;

    public SimpleMotorSimulation(DCMotor gearbox, double gearRatio, double momentOfInertia) {
        super(gearRatio);

        motorSimulation = new ExtendedDCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, momentOfInertia, gearRatio), gearbox);
    }

    public SimpleMotorSimulation(double kv, double ka, DCMotor gearbox, double gearRatio) {
        super(gearRatio);

        motorSimulation = new ExtendedDCMotorSim(LinearSystemId.createDCMotorSystem(kv, ka), gearbox);
    }

    @Override
    public double getCurrent() {
        return motorSimulation.getCurrentDrawAmps();
    }

    @Override
    public double getSystemPositionRotations() {
        return Units.radiansToRotations(motorSimulation.getAngularPositionRad());
    }

    @Override
    public double getSystemVelocityRotationsPerSecond() {
        return Units.radiansToRotations(motorSimulation.getAngularVelocityRadPerSec());
    }

    @Override
    public double getSystemAccelerationRotationsPerSecondSquared() {
        return Units.radiansToRotations(motorSimulation.getAccelerationRadiansPerSecondSquared());
    }

    @Override
    public void setVoltage(double voltage) {
        motorSimulation.setInputVoltage(voltage);
    }

    @Override
    public void updateMotor() {
        motorSimulation.update(ROBOT_PERIODIC_LOOP_TIME);
    }
}