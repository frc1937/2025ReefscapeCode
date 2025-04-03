package frc.robot.subsystems.algaeblaster;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.ELEVATOR;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.*;

public class AlgaeBlaster extends GenericSubsystem {
    private final TrapezoidProfile fastProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(90, 240));

    private final double
            kP = 0.28,
            kV = 0.084973,
            kS = 0.13081;

    private TrapezoidProfile.State previosuState = new TrapezoidProfile.State();

    private int hasSeenCoral = 0;

    @Override
    public void periodic() {
        if (L4_BEAM_BREAK.get() == 0 && ARM_BLASTER_MOTOR.getSystemVelocity() < 15) {
            hasSeenCoral++;
        } else {
            hasSeenCoral = 0;
        }
    }

    @AutoLogOutput(key = "HasCoralInL4")
    public boolean hasCoralInL4Mechanism() {
        return isStill()
                ? hasSeenCoral > 2
                : hasSeenCoral > 5;
    }

    public double getPosition() {
        return ARM_BLASTER_MOTOR.getSystemPosition();
    }

    public boolean isStill() {
        return Math.abs(ARM_BLASTER_MOTOR.getSystemPosition()) < 4;
    }

    public Command algaeBlasterFullThrottle(BlasterArmState target, double trollingVoltage) {
        return new FunctionalCommand(
                () -> previosuState = new TrapezoidProfile.State(ARM_BLASTER_MOTOR.getSystemPosition(),
                        ARM_BLASTER_MOTOR.getSystemVelocity()),
                () -> {
                    final TrapezoidProfile.State targetState = fastProfile.calculate(0.02, previosuState,
                            new TrapezoidProfile.State(target.getRotations(), 0));

                    double output =
                            (targetState.position - ARM_BLASTER_MOTOR.getSystemPosition()) * kP
                                    + targetState.velocity * kV
                                    + kS * Math.signum(targetState.velocity)
                                    + trollingVoltage; //for trolling purposes.

                    Logger.recordOutput("target TARGET", targetState.position);
                    ARM_BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, output);

                    previosuState = targetState;
                },
                interrupt -> {},
                () -> false,
                this
        );
    }

    public Command setMotorVoltage(double voltage) {
        return new FunctionalCommand(
                () -> {},
                () -> ARM_BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage),
                interrupt -> ARM_BLASTER_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command setArmTargetState(AlgaeBlasterConstants.BlasterArmState state) {
        return new FunctionalCommand(
                () -> {},
                () -> ARM_BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotations()),
                interrupt -> ARM_BLASTER_MOTOR.stopMotor(),
                () -> isAtState(state),
                this
);
    }

    public Command setArmStateContinuous(AlgaeBlasterConstants.BlasterArmState state) {
        return new FunctionalCommand(
                () -> {},
                () -> ARM_BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotations()),
                interrupt -> ARM_BLASTER_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }


    public boolean isAtState(BlasterArmState state) {
        return Math.abs(ARM_BLASTER_MOTOR.getSystemPosition() - state.getRotations()) < 0.2
                && ARM_BLASTER_MOTOR.getSystemVelocity() < 10;
    }

    public Command holdAlgaeAtPose(AlgaeBlasterConstants.BlasterArmState state) {
        return new FunctionalCommand(
                () -> {},
                () -> ARM_BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION,state.getRotations()),
                interrupt -> ARM_BLASTER_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command stopAlgaeBlasterArm() {
        return Commands.runOnce(
                ARM_BLASTER_MOTOR::stopMotor, this);
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        ARM_BLASTER_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return BLASTER_SYSID_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        ARM_BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor(ARM_BLASTER_MOTOR.getName())
                .voltage(Volts.of(ARM_BLASTER_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(ARM_BLASTER_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(ARM_BLASTER_MOTOR.getSystemVelocity()));
    }

    public void printPose() {
        if (BLASTER_ARM_MECHANISM != null) {
            final Pose3d current3dPose = new Pose3d(new Translation3d(0.279, 0.31, ELEVATOR.getCurrentHeight() / 2 + 0.81), new Rotation3d(0, getCurrentArmPosition().getRadians() - Math.PI / 2, Math.PI / 2));

            Logger.recordOutput("Components/BlasterArmPose", current3dPose);

            BLASTER_ARM_MECHANISM.updateCurrentAngle(getCurrentArmPosition());
            BLASTER_ARM_MECHANISM.updateTargetAngle(getTargetArmPosition());
        }
    }

    private Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(ARM_BLASTER_MOTOR.getSystemPosition());
    }

    private Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(ARM_BLASTER_MOTOR.getClosedLoopTarget());
    }
}