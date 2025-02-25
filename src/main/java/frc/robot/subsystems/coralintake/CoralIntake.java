package frc.robot.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.coralintake.CoralIntakeConstants.INTAKE_BEAM_BREAK;
import static frc.robot.subsystems.coralintake.CoralIntakeConstants.INTAKE_MOTOR;

public class CoralIntake extends GenericSubsystem {
    public Command prepareGamePiece() {
        return setMotorVoltage(2).until(this::hasCoral);
    }

    public Command releaseGamePiece() {
        return setMotorVoltage(2).until(() -> !hasCoral()).andThen(setMotorVoltage(3)).withTimeout(0.9);
    }

    public Command rotateAlgaeBlasterEndEffector() {
        return setMotorVoltage(-2);
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

    public boolean hasCoral() {
        return INTAKE_BEAM_BREAK.get() == 0;
    }

    private void setVoltage(double voltage) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}