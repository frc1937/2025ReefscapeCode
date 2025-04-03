package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pathfinding.PathfindingConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swerve.SwerveCommands;

import static frc.robot.RobotContainer.*;
import static frc.robot.commands.pathfinding.PathfindingCommands.pathfindToBranch;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.BlasterArmState.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L1;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L4;

public class ConveyorCommands {
    public static Command moveFromIntakeToL4() {
        final ConditionalCommand prepareCoralSlowly = new ConditionalCommand(
                CORAL_INTAKE.setMotorVoltage(-1).onlyWhile(CORAL_INTAKE::hasCoral)
                        .andThen(CORAL_INTAKE.setMotorVoltage(-1.5).withTimeout(0.2))
                        .andThen(CORAL_INTAKE.setMotorVoltage(3).until(CORAL_INTAKE::hasCoral)),

                CORAL_INTAKE.setMotorVoltage(3).until(CORAL_INTAKE::hasCoral)
                        .andThen(CORAL_INTAKE.setMotorVoltage(-1).onlyWhile(CORAL_INTAKE::hasCoral)
                                .andThen(CORAL_INTAKE.setMotorVoltage(-1).withTimeout(0.2)))
                        .andThen(CORAL_INTAKE.setMotorVoltage(2).until(CORAL_INTAKE::hasCoral)),

                CORAL_INTAKE::hasCoral
        );

        final Command finallyRelease = new WaitCommand(1000)
                .until(() -> ALGAE_BLASTER.getPosition() - INTAKE_L4.getRotations() > 0 ||
                        ALGAE_BLASTER.isAtState(INTAKE_L4))
                .andThen(CORAL_INTAKE.releaseToL4Mechanism());

        return new ConditionalCommand(
                Commands.none(),
                (ALGAE_BLASTER.algaeBlasterFullThrottle(INTAKE_L4, 0.5)
                        .withDeadline(prepareCoralSlowly.andThen(finallyRelease))
                        .andThen(
                          CORAL_INTAKE.setMotorVoltage(-0.2)
                                  .withDeadline(ALGAE_BLASTER.setArmTargetState(SCORE_L4_START)))
                ),
                ALGAE_BLASTER::hasCoralInL4Mechanism
        );
    }

    public static Command releaseToL1() {
        return ELEVATOR.setTargetHeight(L1).andThen(ELEVATOR.maintainPosition()).alongWith(
                (ALGAE_BLASTER.setArmStateContinuous(INTAKE_L4)
                .alongWith(CORAL_INTAKE.prepareGamePiece()))
                .until(() -> ALGAE_BLASTER.isAtState(INTAKE_L4))
                .andThen(CORAL_INTAKE.setMotorVoltage(3)
                        .alongWith(new WaitCommand(0.5).andThen(SwerveCommands.driveWithTimeout(-0.12,0,0,true,0.7)))));
    }

    public static Command scoreToL4(PathfindingConstants.Branch branch) {
        final Command setElevatorHeight = ELEVATOR.setTargetHeight(L4).until(() -> ELEVATOR.isAtTargetHeight(L4));

        final Command prepareAlgaeBlaster =
                ALGAE_BLASTER.setArmStateContinuous(SCORE_L4_START)
                        .alongWith(CORAL_INTAKE.setMotorVoltage(0.22)
                                .until(() -> ALGAE_BLASTER.isAtState(SCORE_L4_START)));

        final Command releaseCoral =
                CORAL_INTAKE.compensateForWobblyArm(true)
                        .withDeadline(
                            ALGAE_BLASTER.setArmStateContinuous(SCORE_L4_END)
                            .until(() -> ALGAE_BLASTER.isAtState(SCORE_L4_END))
                        ).andThen(CORAL_INTAKE.scoreToL4());

        return setElevatorHeight.andThen(
                ELEVATOR.maintainPosition()
                        .withDeadline(
                                (prepareAlgaeBlaster
                                .until(()-> SWERVE.isAtPose(branch.getBranchPose(), 0.1, 1.5)))
                                .andThen(releaseCoral)
                                .andThen(ALGAE_BLASTER.setArmTargetState(SCORE_L4_START))
                        )
        );
    }

    public static Command scoreToL4AndPathfind(PathfindingConstants.Branch branch) {
        final Command prepareAlgaeBlaster = moveFromIntakeToL4()
                .andThen(
                        CORAL_INTAKE.compensateForWobblyArm(false)
                                .withDeadline(ALGAE_BLASTER.setArmStateContinuous(SCORE_L4_START))
                        .until(()-> ALGAE_BLASTER.isAtState(SCORE_L4_START)));

        final Command goInFrontOfBranch = pathfindToBranch(branch, new Transform2d(-0.48,0,Rotation2d.kZero));

        final Command pathfindAndReadyElevator = prepareAlgaeBlaster.alongWith(
                        (ELEVATOR.setTargetHeight(L4).until(() -> ELEVATOR.isAtTargetHeight(L4))),
                        goInFrontOfBranch);

        return pathfindAndReadyElevator
                .andThen(SwerveCommands.driveWithTimeout(0.165,0,0,true,0.9)
                        .withDeadline(scoreToL4(branch)))
                .andThen(ALGAE_BLASTER.setArmTargetState(SCORE_L4_START))
                .andThen(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.GO_LOW));
    }
}
