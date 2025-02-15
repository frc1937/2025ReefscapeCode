package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.pathfinding.PathfindingCommands;
import frc.robot.commands.pathfinding.PathfindingConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utilities.FieldConstants;

import static frc.robot.RobotContainer.CORAL_INTAKE;
import static frc.robot.RobotContainer.ELEVATOR;

public class CoralManipulationCommands {
    public static ElevatorConstants.ElevatorHeight CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L2;
    public static boolean SHOULD_BLAST_ALGAE = false;

    public static Command pathfindToBranchAndScore(PathfindingConstants.Branch branch) {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToBranch(branch);

        return (pathfindingCommand
                .alongWith(ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL)))
                .andThen(getAlgaeCommand())
                .andThen(scoreCoralFromCurrentLevel());
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

        return pathfindingCommand.alongWith(eatFromFeeder().withTimeout(3).unless(CORAL_INTAKE::hasCoral));
    }

    public static Command eatFromFeeder() {
        return ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER)
                .alongWith(CORAL_INTAKE.prepareGamePiece());
    }

    public static Command scoreCoralFromCurrentLevel() {
        return ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL).alongWith(
                CORAL_INTAKE.releaseGamePiece().onlyIf(ELEVATOR::isAtTargetPosition).withTimeout(1.5));
    }

    public static Command scoreCoralFromHeight(ElevatorConstants.ElevatorHeight elevatorHeight) {
        return ELEVATOR.setTargetHeight(elevatorHeight).alongWith(
                CORAL_INTAKE.releaseGamePiece().onlyIf(ELEVATOR::isAtTargetPosition)).withTimeout(2);
    }

    private static Command getAlgaeCommand() {
        return new ConditionalCommand(
                AlgaeManipulationCommands.blastAlgaeOffReef().raceWith(new WaitCommand(0.76))
                        .andThen(new InstantCommand(() -> SHOULD_BLAST_ALGAE = false)),
                Commands.none(),
                () -> SHOULD_BLAST_ALGAE
        );
    }
}
