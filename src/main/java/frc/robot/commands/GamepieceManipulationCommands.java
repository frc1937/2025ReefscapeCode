package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.elevator.ElevatorConstants;

import static frc.robot.RobotContainer.CORAL_INTAKE;
import static frc.robot.RobotContainer.ELEVATOR;

public class GamepieceManipulationCommands {
    public static ElevatorConstants.ElevatorHeight CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L2;

    public static Command pathfindToLeftBranchAndScore() {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToLeftBranch();

        return pathfindingCommand
                .alongWith(ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL))
                .until(pathfindingCommand::isFinished)
                .andThen(scoreCoral());
    }

    public static Command pathfindToRightBranchAndScore() {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToRightBranch();

        return pathfindingCommand
                .alongWith(ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL))
                .until(pathfindingCommand::isFinished)
                .andThen(scoreCoral());
    }

    public static Command pathfindToFeederAndEat() {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToFeeder();

        return pathfindingCommand
                .alongWith(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER))
                .until(pathfindingCommand::isFinished).
                andThen(CORAL_INTAKE.prepareGamePiece());
    }

    public static Command scoreCoral() {
        return ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL)
                .until(ELEVATOR::isAtTarget)
                .andThen(CORAL_INTAKE.releaseGamePiece());
    }
}
