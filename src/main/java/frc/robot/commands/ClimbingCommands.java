package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pathfinding.PathfindingCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;

import static frc.robot.RobotContainer.ELEVATOR;

public class ClimbingCommands {
    public static Command pathfindToCageAndClimb() {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToCage();

        return pathfindingCommand
                .alongWith(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L2))
                .until(pathfindingCommand::isFinished)
                .andThen(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.CLIMB));
    }

    public static Command climbCageNoAlignment() {
        return (ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L2))
                .raceWith(new WaitCommand(0.8))
                .andThen(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.CLIMB));
    }
}
