package frc.robot.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.AutoLogOutput;

import static frc.robot.RobotContainer.ALGAE_BLASTER;
import static frc.robot.subsystems.coralintake.CoralIntakeConstants.INTAKE_BEAM_BREAK;
import static frc.robot.subsystems.coralintake.CoralIntakeConstants.INTAKE_MOTOR;

public class CoralIntake extends GenericSubsystem {
    private int hasSeenCoralCounter = 0;

    public Command prepareGamePiece() {
        return setMotorVoltage(1.8).until(this::hasCoral);
    }

    public Command prepareThenTakeBack() {
        return setMotorVoltage(1.8).until(this::hasCoral)
                .andThen(setMotorVoltage(-0.2).withTimeout(0.1));
    }

    public Command prepareWithCorrection() {
        return new ConditionalCommand(
                setMotorVoltage(-3).until(() -> !hasCoral())
                        .andThen(prepareGamePiece()),
                prepareGamePiece(),
                this::hasCoral
        );
    }

    /**
     * Slowly rotate backwards to account for yeetation caused by mechanical incompetence
     */
    public Command compensateForWobblyArm(boolean isRetracting) {
        return setMotorVoltage(isRetracting ? 0 : -0.1);
    }

    public Command releaseToL4Mechanism() {
        return (((setMotorVoltage(2).until(this::hasCoral))
                .andThen(setMotorVoltage(1.42)).until(ALGAE_BLASTER::hasCoralInL4Mechanism)));
    }

    public Command scoreToL4() {
        return setMotorVoltage(-5).withTimeout(0.3); //TODO: HAS CHANGED
    }

    public Command releaseGamePiece() {
        return setMotorVoltage(3).until(() -> !hasCoral())
                .andThen(setMotorVoltage(5)).withTimeout(0.43);
    }

    public Command removeAlgae() {
        return setMotorVoltage(3);
    }

    public Command setMotorVoltage(double voltage) {
        return new FunctionalCommand(
                () -> {},
                () -> setVoltage(voltage),
                (interrupt) -> INTAKE_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command stop() {
        return Commands.runOnce(INTAKE_MOTOR::stopMotor);
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        INTAKE_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public void periodic() {
        if (INTAKE_BEAM_BREAK.get() == 0) {
            hasSeenCoralCounter++;
        } else {
            hasSeenCoralCounter = 0;
        }
    }

    @AutoLogOutput(key="HasCoralInIntake")
    public boolean hasCoral() {
        return hasSeenCoralCounter > 1;
    }

    private void setVoltage(double voltage) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}