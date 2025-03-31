package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.pathfinding.PathfindingCommands;
import frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.FieldConstants;

import java.util.Set;

import static frc.robot.RobotContainer.*;
import static frc.robot.commands.pathfinding.BranchPathfinding.pathfindToTarget;
import static frc.robot.commands.pathfinding.PathfindingCommands.decideReefFace;
import static frc.robot.commands.pathfinding.PathfindingConstants.Branch.CENTER_POSE;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.BlasterArmState.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;
import static frc.robot.subsystems.swerve.SwerveCommands.driveWithTimeout;

public class CoralManipulationCommands {
    public static ElevatorConstants.ElevatorHeight CURRENT_SCORING_LEVEL = L1;

    public static Command pathfindToFeederAndEat() {
        return PathfindingCommands.pathfindToFeeder()
                .alongWith(eatFromFeeder());
    }

    //WORKS!
    public static Command yeetAlgaeWithAlignment() {
        final Transform2d reefToAlgaeStartPoseLocation = (new Transform2d(-0.76, 0, Rotation2d.kZero));

        final Command driveToStartPoseAlgae = (pathfindToTarget(() -> CENTER_POSE.getBranchPose().transformBy(reefToAlgaeStartPoseLocation),0.25));

        final Command driveToAlgae =
                new DeferredCommand(
                        () -> SwerveCommands.goToPosePID(CENTER_POSE.getBranchPose()
                                .transformBy(new Transform2d(0, 0.0, Rotation2d.kZero))),
                        Set.of(SWERVE)
                );

        final ConditionalCommand kick = new ConditionalCommand(
                ELEVATOR.setTargetHeight(L1).andThen( //BLASTR FROM L3
                        ELEVATOR.maintainPosition().withDeadline(
                        (ALGAE_BLASTER.algaeBlasterFullThrottle(HORIZONTAL_OUT, 0.1)
                        .until(() -> SWERVE.isAtPose(CENTER_POSE.getBranchPose(), 0.2, 5))
                        .andThen(ALGAE_BLASTER.algaeBlasterFullThrottle(DEFAULT_POSE, -0.1)
                                .withTimeout(1))))),

                ELEVATOR.setTargetHeight(FEEDER).withDeadline( //BLAST FROM L2
                        (((ALGAE_BLASTER.algaeBlasterFullThrottle(INTAKE_L4, 0.01))
                         .until(() -> SWERVE.isAtPose(CENTER_POSE.getBranchPose(), 0.32, 8))
                        ).andThen(
                                 ALGAE_BLASTER.algaeBlasterFullThrottle(SCORE_L4_START, -0.01)
                                                .withTimeout(0.4)))
                ),
                () -> decideReefFace().ordinal() % 2 == 0
                );

        return ((driveToStartPoseAlgae.andThen(driveToAlgae))
                .alongWith(kick))
                .andThen(driveWithTimeout(-0.2, 0, 0, true, 1.3));
    }
    /**
     * Pathfinds to the specified feeder while constantly eating. Stops when the coral intake has coral
     *
     * @param feeder The feeder to pathfind to
     * @return The command
     */
    public static Command pathfindToFeederAndEat(FieldConstants.Feeder feeder) {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToFeederBezier(feeder);

        return pathfindingCommand
                .alongWith(eatFromFeederAutonomous());
    }

    public static Command eatFromFeederAutonomous() {
        return (ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER)
                .until(() -> ELEVATOR.isAtTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER))
                .alongWith(CORAL_INTAKE.setMotorVoltage(3.2)))
                .alongWith(ALGAE_BLASTER.holdAlgaeAtPose(AlgaeBlasterConstants.BlasterArmState.VERTICAL))
                .until(CORAL_INTAKE::hasCoral);
    }

    public static Command eatFromFeeder() {
        return (ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER)
                .until(() -> ELEVATOR.isAtTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER))
                .alongWith(CORAL_INTAKE.prepareGamePiece()))
                .alongWith(ALGAE_BLASTER.holdAlgaeAtPose(AlgaeBlasterConstants.BlasterArmState.VERTICAL))
                .until(CORAL_INTAKE::hasCoral)
                .alongWith(LEDS.setLEDStatus(Leds.LEDMode.EATING, 3));
    }

    public static Command yeetAlgaeNeverStops(ElevatorConstants.ElevatorHeight height) {
        return ELEVATOR.setTargetHeight(() -> height)
                .until(() -> ELEVATOR.isAtTargetHeight(height))
                .andThen(
                        ALGAE_BLASTER.setArmStateContinuous(HORIZONTAL_OUT)
                                .alongWith(CORAL_INTAKE.removeAlgae())
                                .alongWith(ELEVATOR.maintainPosition())
                );
    }

    public static Command justReleaseACoralTeleop() {
        return new ConditionalCommand(
                (ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL)
                        .until(() -> ELEVATOR.isAtTargetHeight(CURRENT_SCORING_LEVEL))
                        .andThen(releaseCoral())),

                ELEVATOR.setTargetHeight(L1)
                        .until(() -> ELEVATOR.isAtTargetHeight(L1))
                        .andThen(CORAL_INTAKE.setMotorVoltage(3.4)),

                new Trigger(() -> CURRENT_SCORING_LEVEL.getRotations() != L1.getRotations())
        );

    }

    public static Command scoreCoralFromHeight(ElevatorConstants.ElevatorHeight elevatorHeightThing) {
        return (ELEVATOR.setTargetHeight(elevatorHeightThing)
                .until(() -> ELEVATOR.isAtTargetHeight(elevatorHeightThing)))
                .andThen(ELEVATOR.maintainPosition().withDeadline(CORAL_INTAKE.releaseGamePiece()));
    }

    public static Command releaseCoral() {
        return ((CORAL_INTAKE.releaseGamePiece()))
                .alongWith();
    }
}
