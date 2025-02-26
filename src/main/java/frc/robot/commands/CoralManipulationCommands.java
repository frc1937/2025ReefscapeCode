package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.pathfinding.PathfindingCommands;
import frc.robot.commands.pathfinding.PathfindingConstants;
import frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utilities.FieldConstants;

import static frc.robot.RobotContainer.*;

public class CoralManipulationCommands {
    public static ElevatorConstants.ElevatorHeight CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L3;
    public static boolean SHOULD_BLAST_ALGAE = false;
    public static final Trigger shouldBlastAlgae = new Trigger(() -> SHOULD_BLAST_ALGAE);

    public static Command pathfindToBranchAndScoreForTeleop(PathfindingConstants.Branch branch) {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToBranch(branch);

        return pathfindingCommand
                .alongWith(ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL))
                .andThen(scoreCoralFromCurrentLevelAndBlastAlgaeForTeleop());
    }

    public static Command pathfindToFeederAndEat() {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToFeeder();

        return pathfindingCommand.alongWith(eatFromFeeder());
    }

    /**
     * Pathfinds to the specified feeder while constantly eating. Stops when the coral intake has coral
     *
     * @param feeder The feeder to pathfind to
     * @return The command
     */
    public static Command pathfindToFeederAndEat(FieldConstants.Feeder feeder) {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToFeederBezier(feeder);

        return pathfindingCommand.alongWith(eatFromFeeder());
    }

    public static Command eatFromFeeder() {
        return ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER)
                        .until(() -> ELEVATOR.isAtTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER))
                        .andThen(ALGAE_BLASTER.holdAlgaeAtPose(AlgaeBlasterConstants.BlasterArmState.VERTICAL))
                        .alongWith(CORAL_INTAKE.prepareGamePiece()).until(CORAL_INTAKE::hasCoral);
    }

    public static Command scoreCoralFromCurrentLevelAndBlastAlgaeForTeleop() {
        final ConditionalCommand optionallySpitAlgae = new ConditionalCommand(
                ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_OUT)
                        .alongWith(CORAL_INTAKE.rotateAlgaeBlasterEndEffector())
                        .until(shouldBlastAlgae.negate()),
                Commands.none(),
                shouldBlastAlgae);

        return ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL)
                .until(() -> ELEVATOR.isAtTargetHeight(CURRENT_SCORING_LEVEL))
                .andThen(optionallySpitAlgae)
                .andThen(CORAL_INTAKE.releaseGamePiece());
    }

    public static Command scoreCoralFromCurrentLevelAndBlastAlgae() {
        final ConditionalCommand optionallySpitAlgae = new ConditionalCommand(
                ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_OUT)
                        .alongWith(CORAL_INTAKE.rotateAlgaeBlasterEndEffector()),
                Commands.none(),
                shouldBlastAlgae);

        final ConditionalCommand optionallyRetractAlgae = new ConditionalCommand(
                ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN)
                        .alongWith(new InstantCommand(() -> SHOULD_BLAST_ALGAE = false)),
                Commands.none(),
                shouldBlastAlgae
        );

        return ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL)
                .until(() -> ELEVATOR.isAtTargetHeight(CURRENT_SCORING_LEVEL))
                .andThen(optionallySpitAlgae)
                .andThen(CORAL_INTAKE.releaseGamePiece())
                .andThen(optionallyRetractAlgae);
    }

    public static Command scoreCoralFromHeight(ElevatorConstants.ElevatorHeight elevatorHeight) {
        return ELEVATOR.setTargetHeight(elevatorHeight)
                .alongWith(CORAL_INTAKE.prepareGamePiece())
                .andThen(CORAL_INTAKE.releaseGamePiece());
    }

    private static Command getAlgaeCommand() {
        return new ConditionalCommand(
                AlgaeManipulationCommands.blastAlgaeOffReef()
                        .raceWith(new WaitCommand(0.76))
                        .andThen(new InstantCommand(() -> SHOULD_BLAST_ALGAE = false)),
                Commands.none(),
                () -> SHOULD_BLAST_ALGAE
        );
    }
}
