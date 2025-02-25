package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.flippable.FlippableRotation2d;
import frc.robot.commands.pathfinding.PathfindingCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveConstants;

import static frc.robot.RobotContainer.ELEVATOR;

public class ClimbingCommands {
    public static Command pathfindToCageAndClimb() {
        final DeferredCommand pathfindingCommand = PathfindingCommands.pathfindToCage();

        return pathfindingCommand
                .alongWith(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L2))
                .until(pathfindingCommand::isFinished)
                .andThen(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.CLIMB_LOWERING));
    }

    public static Command climbCageNoAlignment() {
        return (ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L2))
                .raceWith(new WaitCommand(1.8))
                .andThen(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.CLIMB_LOWERING));
    }

    //UNTESTED
    public static Command rotateToCage() {
        //PUT elevator at L2
        //Rotate so elevator is facing the cage (FROM BEHIND)
        //Drive forward a bit.
        //drive backwards a bit
        //drive elevator downwards.
        return SwerveCommands.rotateToTarget(new FlippableRotation2d(Rotation2d.kPi, true))
                .alongWith(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L2))
                .andThen(SwerveCommands.driveOpenLoop(() -> 0.5, () -> 0, () -> 0, () -> true))
                .raceWith(new WaitCommand(0.8))
                .andThen(SwerveCommands.driveOpenLoop(() -> -0.5, () -> 0, () -> 0, () -> true))
                .raceWith(new WaitCommand(0.5))
                .andThen(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.CLIMB_LOWERING));
    }
}
