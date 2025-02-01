package frc.lib.generic;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.hardware.motor.MotorProperties;

import static edu.wpi.first.units.Units.Volts;

public abstract class GenericSubsystem extends SubsystemBase {
    private final SysIdRoutine routine = generateSysIdRoutine();

    /**
     * Creates a quasistatic (ramp up) command for characterizing the subsystem's mechanism.
     *
     * @param direction the direction in which to run the test
     * @return the command
     * @throws IllegalStateException if the {@link GenericSubsystem#getSysIdConfig()} function wasn't overridden or returns null
     */
    public final Command getSysIdQuastatic(SysIdRoutine.Direction direction) throws IllegalStateException {
        if (routine == null)
            new IllegalStateException("Subsystem " + getName() + " doesn't have a SysId routine!").printStackTrace();

        return routine.quasistatic(direction);
    }

    /**
     * Creates a dynamic (constant "step up") command for characterizing the subsystem's mechanism.
     *
     * @param direction the direction in which to run the test
     * @return the command
     * @throws IllegalStateException if the {@link GenericSubsystem#getSysIdConfig()} function wasn't overridden or returns null
     */
    public final Command getSysIdDynamic(SysIdRoutine.Direction direction) throws IllegalStateException {
        if (routine == null)
            new IllegalStateException("Subsystem " + getName() + " doesn't have a SysId routine!").printStackTrace();

        return routine.dynamic(direction);
    }

    public SysIdRoutine.Config getSysIdConfig() {
        return null;
    }

    public void sysIdDrive(double voltage) {
    }

    public void sysIdUpdateLog(SysIdRoutineLog log) {
    }

    public void setIdleMode(MotorProperties.IdleMode idleMode) {}

    private SysIdRoutine generateSysIdRoutine() {
        if (getSysIdConfig() == null)
            return null;

        return new SysIdRoutine(
                getSysIdConfig(),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> sysIdDrive(voltageMeasure.in(Volts)),
                        this::sysIdUpdateLog,
                        this,
                        getName()
                )
        );
    }
}
