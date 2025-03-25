package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.pathfinding.BranchPathfinding;
import frc.robot.commands.pathfinding.PathfindingConstants.Branch;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.FieldConstants.Feeder;
import frc.robot.utilities.FieldConstants.ReefFace;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static frc.robot.RobotContainer.*;
import static frc.robot.commands.ConveyorCommands.moveFromIntakeToL4;
import static frc.robot.commands.ConveyorCommands.scoreToL4;
import static frc.robot.commands.CoralManipulationCommands.pathfindToFeederAndEat;
import static frc.robot.commands.CoralManipulationCommands.scoreCoralFromHeight;
import static frc.robot.commands.pathfinding.BranchPathfinding.L4DistanceFromReef;
import static frc.robot.commands.pathfinding.PathfindingCommands.pathfindToBranchBezier;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.BlasterArmState.SCORE_L4_START;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L2;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L4;
import static frc.robot.subsystems.swerve.SwerveCommands.driveWithTimeout;
import static frc.robot.subsystems.swerve.SwerveCommands.goToPosePID;
import static frc.robot.utilities.FieldConstants.ReefFace.FACE_0;

public class Questionnaire {
    private final LoggedDashboardChooser<String> PRESET_QUESTION;
    private final Cycle
            CYCLE_1,
            CYCLE_2,
            CYCLE_3;

    public Questionnaire() {
        PRESET_QUESTION = createPresetQuestion();

        CYCLE_1 = initializeCycleFromKey("1");
        CYCLE_2 = initializeCycleFromKey("2");
        CYCLE_3 = initializeCycleFromKey("3");
    }

    private Cycle initializeCycleFromKey(String key) {
        return new Cycle(createReefFaceQuestion(key),
                createBranchQuestion(key),
                createScoringQuestion(key),
                createFeederQuestion(key));
    }

    private LoggedDashboardChooser<String> createPresetQuestion() {
        final LoggedDashboardChooser<String> question = new LoggedDashboardChooser<>("Which Auto?");

        question.addDefaultOption("Selectable Path", "Selectable Path");
        question.addOption("TryL1", "TryL1");
        question.addOption("Middle L4x1 LEFT", "Middle L4x1 LEFT");
        question.addOption("Middle L4x1 RIGHT", "Middle L4x1 RIGHT");

        return question;
    }

    private LoggedDashboardChooser<ReefFace> createReefFaceQuestion(String cycleNumber) {
        final LoggedDashboardChooser<ReefFace> question = new LoggedDashboardChooser<>(cycleNumber + "Which Reef Face?");

        question.addDefaultOption("#" + cycleNumber+ " Face 0", FACE_0);

        for (ReefFace face : ReefFace.values()) {
            if (face == FACE_0) continue;

            question.addOption("#" + cycleNumber+ " Face " + face.ordinal(), face);
        }

        return question;
    }

    private LoggedDashboardChooser<Branch> createBranchQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Branch> question = new LoggedDashboardChooser<>(cycleNumber + "Which Branch?");

        question.addDefaultOption("#" + cycleNumber+ " Left", Branch.LEFT_BRANCH);
        question.addOption("#" + cycleNumber+ " Right", Branch.RIGHT_BRANCH);

        return question;
    }

    private LoggedDashboardChooser<ElevatorConstants.ElevatorHeight> createScoringQuestion(String cycleNumber) {
        final LoggedDashboardChooser<ElevatorConstants.ElevatorHeight> question = new LoggedDashboardChooser<>(cycleNumber + "Which Scoring Level?");

        question.addDefaultOption("#" + cycleNumber+ " L4", L4);
        question.addOption("#" + cycleNumber+ " L2", L2);

        return question;
    }

    private LoggedDashboardChooser<Command> createFeederQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + "Which Feeder?");

        question.addDefaultOption("#" + cycleNumber+ " Blue's LEFT Red's RIGHT Feeder", pathfindToFeederAndEat(Feeder.BLUES_LEFT_REDS_RIGHT_FEEDER));
        question.addOption("#" + cycleNumber+ " Blue's RIGHT Red's LEFT Feeder", pathfindToFeederAndEat(Feeder.BLUES_RIGHT_REDS_LEFT_FEEDER));

        return question;
    }

    private Command createCycleSequence(Cycle cycle) {
        final ReefFace selectedReefFace = cycle.reefFaceQuestion.get();
        final Branch selectedBranch = cycle.branchQuestion.get();
        final ElevatorConstants.ElevatorHeight scoringHeight = cycle.scoringHeightQuestion.get();

        if (scoringHeight != L4) {
            final Command goToBranch =
                    pathfindToBranchBezier(selectedBranch, selectedReefFace, new Transform2d());
            final Command readyTheElevator = ELEVATOR.setTargetHeight(L2)
                    .andThen(ELEVATOR.maintainPosition());

            return ((readyTheElevator.alongWith(CORAL_INTAKE.prepareWithCorrection())).withDeadline(goToBranch))
                    .andThen(scoreCoralFromHeight(scoringHeight))
                    .andThen((cycle.feederQuestion.get()));
        }

        final Command goInFrontOfBranch = pathfindToBranchBezier(selectedBranch, selectedReefFace, L4DistanceFromReef);
        final Pose2d targetPose = selectedBranch.getBranchPose(selectedReefFace).transformBy(L4DistanceFromReef);

        final Command moveToIntake = moveFromIntakeToL4()
                .andThen(CORAL_INTAKE.setMotorVoltage(1.3).withTimeout(0.18));

        final Command moveBlasterToScorePosition =
                (ALGAE_BLASTER.setArmStateContinuous(SCORE_L4_START)
                        .alongWith(CORAL_INTAKE.setMotorVoltage(0.3)))
                .until(() -> ALGAE_BLASTER.isAtState(SCORE_L4_START) && ALGAE_BLASTER.hasCoralInL4Mechanism());

        return (((goInFrontOfBranch
                .alongWith(ELEVATOR.setTargetHeight(L4.getMiddlePoint()))
                .until(() -> SWERVE.isAtPose(targetPose, 0.2, 1)))
                ).alongWith(moveToIntake.andThen((moveBlasterToScorePosition))))
                .andThen(goToPosePID(targetPose.transformBy(L4DistanceFromReef.inverse()))
                                .withDeadline(scoreToL4(selectedBranch)))

                .andThen(driveWithTimeout(-0.165,0,0,true,0.2))

                .andThen(cycle.feederQuestion.get());
    }

    public Command getCommand() {
        if (PRESET_QUESTION.getSendableChooser().getSelected() == "TryL1") {
            return SwerveCommands.driveOpenLoop(() -> 0.2, () -> 0, () -> 0, () -> true)
                    .withTimeout(8)
                    .andThen(SwerveCommands.driveOpenLoop(() -> 0, () -> 0, () -> 0, () -> true))
                    .raceWith(new WaitCommand(0.5))
                    .andThen(new WaitCommand(2))
                    .andThen(CORAL_INTAKE.releaseGamePiece());
        }

        if (PRESET_QUESTION.getSendableChooser().getSelected() == "Middle L4x1 LEFT") {
            return BranchPathfinding.pathAndScoreWithOverride(Branch.LEFT_BRANCH,
                    () -> 0, () -> 0, () -> 0,
                    new Trigger(() -> false));
        }

        if (PRESET_QUESTION.getSendableChooser().getSelected() == "Middle L4x1 RIGHT") {
            return BranchPathfinding.pathAndScoreWithOverride(Branch.RIGHT_BRANCH, () -> 0, () -> 0, () -> 0,
                    new Trigger(() -> false));
        }

        return Commands.sequence(
                createCycleSequence(CYCLE_1),
                createCycleSequence(CYCLE_2),
                createCycleSequence(CYCLE_3)
        );
    }

    public String getSelected() {
        return PRESET_QUESTION.getSendableChooser().getSelected() != "None" ? PRESET_QUESTION.get() : "Custom";
    }

    private record Cycle(
            LoggedDashboardChooser<ReefFace> reefFaceQuestion,
            LoggedDashboardChooser<Branch> branchQuestion,
            LoggedDashboardChooser<ElevatorConstants.ElevatorHeight> scoringHeightQuestion,
            LoggedDashboardChooser<Command> feederQuestion) {
    }
}