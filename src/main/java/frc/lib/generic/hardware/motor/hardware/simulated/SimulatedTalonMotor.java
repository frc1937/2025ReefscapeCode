package frc.lib.generic.hardware.motor.hardware.simulated;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.hardware.motor.MotorSignal;
import frc.lib.generic.hardware.signals.ctre.CTREInputs;
import frc.lib.generic.simulation.GenericPhysicsSimulation;
import frc.robot.GlobalConstants;

import static frc.lib.generic.hardware.motor.MotorProperties.GravityType.ARM;
import static frc.robot.GlobalConstants.CURRENT_MODE;

public class SimulatedTalonMotor extends Motor {
    private final CTREInputs inputs;

    private MotorConfiguration currentConfiguration;
    private GenericPhysicsSimulation simulation;

    private final TalonFX talonFX;
    private final TalonFXSimState talonFXSimState;

    private final StatusSignal<Voltage> voltageSignal;
    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator talonConfigurator;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage positionMMRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityMMRequest = new MotionMagicVelocityVoltage(0);

    private boolean shouldUseProfile = false;
    private double target = 0;

    public SimulatedTalonMotor(String name, int port) {
        super(name);

        talonFX = new TalonFX(port);

        talonFXSimState = talonFX.getSimState();
        talonFXSimState.setSupplyVoltage(12);

        talonConfigurator = talonFX.getConfigurator();

        voltageSignal = talonFX.getMotorVoltage().clone();
        voltageSignal.setUpdateFrequency(1000);

        if (CURRENT_MODE != GlobalConstants.Mode.SIMULATION)
            new RuntimeException("DO NOT Initialize THIS MOTOR! Use the factory methods instead!").printStackTrace();

        inputs = new CTREInputs(name);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output) {
        target = output;

        switch (mode) {
            case VOLTAGE -> talonFX.setControl(voltageRequest.withOutput(output));

            case POSITION -> {
                if (shouldUseProfile) {
                    talonFX.setControl(positionMMRequest.withPosition(output).withSlot(0));
                } else {
                    talonFX.setControl(positionVoltageRequest.withPosition(output).withSlot(0));
                }
            }

            case VELOCITY -> {
                if (shouldUseProfile) {
                    talonFX.setControl(velocityMMRequest.withVelocity(output).withSlot(0));
                } else {
                    talonFX.setControl(velocityVoltageRequest.withVelocity(output).withSlot(0));
                }
            }

            case CURRENT ->
                    new UnsupportedOperationException("CTRE LOVES money and wants $150!!! dollars for this.. wtf.").printStackTrace();
        }
    }

    @Override
    public void stopMotor() {
        talonFX.stopMotor();
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        simulation = configuration.simulationProperties.getSimulationType();

        if (simulation == null) {
            new RuntimeException("Simulation configurations not found for motor: " + getName()).printStackTrace();
            return false;
        }

        talonConfig.Feedback.SensorToMechanismRatio = configuration.gearRatio;

        configureMotionMagic();
        configurePIDSlot();

        applySoftwarePositionLimits();

        talonConfig.ClosedLoopGeneral.ContinuousWrap = configuration.closedLoopContinuousWrap;

        return talonConfigurator.apply(talonConfig) == StatusCode.OK;
    }

    private void configureMotionMagic() {
        if (currentConfiguration.profileMaxVelocity == 0 && currentConfiguration.profileMaxAcceleration == 0 && currentConfiguration.profileMaxJerk == 0 ||
                currentConfiguration.profileMaxVelocity != 0 && currentConfiguration.profileMaxAcceleration == 0 && currentConfiguration.profileMaxJerk == 0)
            return;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = currentConfiguration.profileMaxVelocity;
        talonConfig.MotionMagic.MotionMagicAcceleration = currentConfiguration.profileMaxAcceleration;
        talonConfig.MotionMagic.MotionMagicJerk = currentConfiguration.profileMaxJerk;

        shouldUseProfile = true;
    }

    private void applySoftwarePositionLimits() {
        if (currentConfiguration.forwardSoftLimit != null) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = currentConfiguration.forwardSoftLimit;
        }

        if (currentConfiguration.reverseSoftLimit != null) {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = currentConfiguration.reverseSoftLimit;
        }
    }

    private void configurePIDSlot() {
        talonConfig.Slot0.kP = currentConfiguration.simulationSlot.kP();
        talonConfig.Slot0.kI = currentConfiguration.simulationSlot.kI();
        talonConfig.Slot0.kD = currentConfiguration.simulationSlot.kD();

        talonConfig.Slot0.kA = currentConfiguration.simulationSlot.kA();
        talonConfig.Slot0.kS = currentConfiguration.simulationSlot.kS();
        talonConfig.Slot0.kV = currentConfiguration.simulationSlot.kV();
        talonConfig.Slot0.kG = currentConfiguration.simulationSlot.kG();

        if (currentConfiguration.simulationSlot.gravityType() != null)
            talonConfig.Slot0.GravityType = currentConfiguration.simulationSlot.gravityType() == ARM ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
    }

    @Override
    public int getDeviceID() {
        return talonFX.getDeviceID();
    }

    @Override
    public void registerSignal(MotorSignal signal, boolean useFasterThread) {
        switch (signal) {
            case VOLTAGE -> inputs.registerCTRESignal(signal, voltageSignal, 50);
            case CURRENT -> inputs.registerSupplierForSimulation(MotorSignal.CURRENT, () -> simulation.getCurrent());
            case TEMPERATURE -> inputs.registerSupplierForSimulation(MotorSignal.TEMPERATURE, () -> 0);
            case CLOSED_LOOP_TARGET -> inputs.registerSupplierForSimulation(MotorSignal.CLOSED_LOOP_TARGET, () -> target);
            case POSITION -> inputs.registerSupplierForSimulation(MotorSignal.POSITION, this::getSimulationPosition);
            case VELOCITY -> inputs.registerSupplierForSimulation(MotorSignal.VELOCITY, this::getSimulationVelocity);
            case ACCELERATION -> inputs.registerSupplierForSimulation(MotorSignal.ACCELERATION, this::getSimulationAcceleration);
        }
    }

    public void updateSimulation() {
        simulation.setVoltage(talonFXSimState.getMotorVoltage());
        simulation.updateMotor();

        talonFXSimState.setRawRotorPosition(simulation.getMotorPositionRotations());
        talonFXSimState.setRotorVelocity(simulation.getMotorVelocityRotationsPerSecond());
        talonFXSimState.setRotorAcceleration(simulation.getMotorAccelerationRotationsPerSecondSquared());
    }

    private double getSimulationAcceleration() {
        if (simulation == null) return 0;
        return simulation.getSystemAccelerationRotationsPerSecondSquared();
    }

    private double getSimulationVelocity() {
        if (simulation == null) return 0;
        return simulation.getSystemVelocityRotationsPerSecond();
    }

    private double getSimulationPosition() {
        if (simulation == null) return 0;
        return simulation.getSystemPositionRotations();
    }

    @Override
    public CTREInputs getInputs() {
        return inputs;
    }
}