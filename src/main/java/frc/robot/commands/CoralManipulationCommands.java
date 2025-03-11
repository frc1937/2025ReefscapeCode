package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.pathfinding.PathfindingCommands;
import frc.robot.commands.pathfinding.PathfindingConstants;
import frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.utilities.FieldConstants;

import static frc.robot.RobotContainer.*;
import static frc.robot.commands.pathfinding.PathfindingCommands.pathfindToBranch;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_OUT;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L3;

public class CoralManipulationCommands {
    public static ElevatorConstants.ElevatorHeight CURRENT_SCORING_LEVEL = L3;

    public static Command pathfindToBranchAndScoreForTeleop(PathfindingConstants.Branch branch) {
        final ParallelDeadlineGroup pathfindAndReadyElevator = new ParallelDeadlineGroup(
                pathfindToBranch(branch)
                .alongWith((ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL)
                .until(() -> ELEVATOR.isAtTargetHeight(CURRENT_SCORING_LEVEL))),

                ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN))
        );

        return pathfindAndReadyElevator
                .andThen(justReleaseACoral())
                .andThen(
                        (ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.GO_LOW))
                );

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

        return pathfindingCommand
                .alongWith(eatFromFeederAutonomous());
    }

    public static Command eatFromFeederAutonomous() {
        return (ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER)
                .until(() -> ELEVATOR.isAtTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER))
                .alongWith(CORAL_INTAKE.prepareGamePiece()))
                .alongWith(ALGAE_BLASTER.holdAlgaeAtPose(AlgaeBlasterConstants.BlasterArmState.VERTICAL))
                .until(CORAL_INTAKE::hasCoral)
                .andThen(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN));
    }

    public static Command eatFromFeeder() {
        return (ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER)
                .until(() -> ELEVATOR.isAtTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER))
                .alongWith(CORAL_INTAKE.prepareGamePiece()))
                .alongWith(ALGAE_BLASTER.holdAlgaeAtPose(AlgaeBlasterConstants.BlasterArmState.VERTICAL))
                .until(CORAL_INTAKE::hasCoral)
                .andThen(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN))
                .alongWith(LEDS.setLEDStatus(Leds.LEDMode.EATING, 3));
    }

    public static Command yeetAlgaeNeverStops(ElevatorConstants.ElevatorHeight height) {
        return ELEVATOR.setTargetHeight(() -> height)
                .until(() -> ELEVATOR.isAtTargetHeight(height))
                .andThen(
                        ALGAE_BLASTER.setAlgaeBlasterArmState(HORIZONTAL_OUT)
                                .alongWith(CORAL_INTAKE.rotateAlgaeBlasterEndEffector())
                                .alongWith(ELEVATOR.maintainPosition())
                );
    }

    public static Command retractAlgaeArm() {
        return ALGAE_BLASTER.setAlgaeBlasterArmState(HORIZONTAL_IN)
                .alongWith(CORAL_INTAKE.stop());
    }

    public static Command justReleaseACoral() {
        return (ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL)
                                .until(() -> ELEVATOR.isAtTargetHeight(CURRENT_SCORING_LEVEL))
                                .andThen(releaseCoral()));
    }

    public static Command scoreCoralFromHeight(ElevatorConstants.ElevatorHeight elevatorHeightThing) {
        return (ELEVATOR.setTargetHeight(elevatorHeightThing)
                .until(() -> ELEVATOR.isAtTargetHeight(elevatorHeightThing)))
                .andThen(CORAL_INTAKE.releaseGamePiece());
        //                .andThen(CORAL_INTAKE.setMotorVoltage(9).alongWith(ELEVATOR.maintainPosition())));
//        return (ELEVATOR.setTargetHeight(elevatorHeight).raceWith(new WaitCommand(2)))
//                .andThen(CORAL_INTAKE.setMotorVoltage(3).withDeadline(new WaitCommand(2))
//                .andThen(CORAL_INTAKE.stop().withDeadline(new WaitCommand(2))))
//                .andThen(new InstantCommand(()-> System.out.println("Yo Nigg")).withTimeout(2));
    }

    public static Command releaseCoral() {
        return CORAL_INTAKE.releaseGamePiece().alongWith(ELEVATOR.maintainPosition());
    }
}
