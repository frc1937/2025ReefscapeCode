package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utilities.FieldConstants;

import static frc.robot.RobotContainer.CORAL_INTAKE;
import static frc.robot.RobotContainer.ELEVATOR;

public class CoralManipulationCommands {
    public static ElevatorConstants.ElevatorHeight CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L2;
    public static boolean SHOULD_BLAST_ALGAE = false;

    public static Command pathfindToLeftBranchAndScore() {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToLeftBranch();

        return pathfindingCommand
                .alongWith(ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL))
                .until(pathfindingCommand::isFinished)
                .andThen(getAlgaeCommand())
                .andThen(scoreCoralNoPositionCheck());
    }

    public static Command pathfindToRightBranchAndScore() {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToRightBranch();

        return pathfindingCommand
                .alongWith(ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL))
                .until(pathfindingCommand::isFinished)
                .andThen(getAlgaeCommand())
                .andThen(scoreCoralNoPositionCheck());
    }

    public static Command pathfindToFeederAndEat() {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToFeeder();

        return pathfindingCommand.alongWith(eatFromFeeder());
    }

    public static Command pathfindToFeederAndEat(FieldConstants.Feeder feeder) {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToFeeder(feeder);

        return pathfindingCommand.alongWith(eatFromFeeder().withTimeout(3).unless(CORAL_INTAKE::hasCoral));
    }

    public static Command eatFromFeeder() {
        return ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER)
                .alongWith(CORAL_INTAKE.prepareGamePiece());
    }

    public static Command scoreCoralNoPositionCheck() {
        return ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL)
                .until(ELEVATOR::isAtTarget)
                .andThen(CORAL_INTAKE.releaseGamePiece());
    }
 
    public static Command scoreGamePiece(ElevatorConstants.ElevatorHeight elevatorHeight) {
        return ELEVATOR.setTargetHeight(elevatorHeight).andThen(
                CORAL_INTAKE.releaseGamePiece()).andThen(CORAL_INTAKE.stop());
    }

    private static Command getAlgaeCommand() {
        return new ConditionalCommand(
                AlgaeManipulationCommands.blastAlgaeOffReef().raceWith(new WaitCommand(0.76))
                        .andThen(new InstantCommand(() -> SHOULD_BLAST_ALGAE = false)),
                Commands.none(),
                () -> SHOULD_BLAST_ALGAE
        );
}
