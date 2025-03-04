package frc.robot.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.AutoLogOutput;

import static frc.robot.subsystems.coralintake.CoralIntakeConstants.INTAKE_BEAM_BREAK;
import static frc.robot.subsystems.coralintake.CoralIntakeConstants.INTAKE_MOTOR;

public class CoralIntake extends GenericSubsystem {
    private int hasSeenCoralCounter = 0;

    public Command prepareGamePiece() {
        return setMotorVoltage(2.6).until(this::hasCoral);
    }

    public Command releaseGamePiece() {
        return setMotorVoltage(2).until(() -> !hasCoral()).andThen(setMotorVoltage(3))
                .withTimeout(0.9);
    }

    public Command rotateAlgaeBlasterEndEffector() {
        return setMotorVoltage(-3);
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

    @AutoLogOutput(key="HasCoral")
    public boolean hasCoral() {
        return hasSeenCoralCounter > 1;
    }

    private void setVoltage(double voltage) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}