package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.StaticFrictionCharacterization;
import frc.lib.generic.characterization.WheelRadiusCharacterization;
import frc.lib.generic.hardware.controllers.Controller;
import frc.lib.generic.hardware.controllers.KeyboardController;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.ClimbingCommands;
import frc.robot.commands.CoralManipulationCommands;
import frc.robot.commands.pathfinding.PathfindingCommands;
import frc.robot.commands.pathfinding.PathfindingConstants;
import frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.FieldConstants;

import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_X;
import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_Y;
import static frc.robot.RobotContainer.*;
import static frc.robot.commands.CoralManipulationCommands.SHOULD_BLAST_ALGAE;
import static frc.robot.subsystems.swerve.SwerveCommands.rotateToTarget;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class ButtonControls {
    public static final DoubleSupplier DRIVE_SIGN = () -> Flippable.isRedAlliance() ? 1 : -1;

    public enum ButtonLayout {
        DEVELOPMENT,
        ELEVATOR_KS,
        TELEOP,
        ALGAE_BLASTER_KS,
        CHARACTERIZE_ELEVATOR,
        CHARACTERIZE_SWERVE_DRIVE_MOTORS,
        CHARACTERIZE_WHEEL_RADIUS,
        CHARACTERIZE_ALGAE_BLASTER_ARM,
        CHARACTERIZE_ALGAE_INTAKE_ARM,
        CHARACTERIZE_SWERVE_AZIMUTH,
        PATHPLANNER_TEST
    }

    private static final Controller DRIVER_CONTROLLER = new Controller(0);
    private static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    private static final Trigger USER_BUTTON = new Trigger(RobotController::getUserButton);

    public static void initializeButtons(ButtonLayout layout) {
        setupUserButtonDebugging();

        switch (layout) {
            case TELEOP -> configureButtonsTeleop();
            case ELEVATOR_KS -> characterizeElevatorKSandKG();
            case DEVELOPMENT -> configureButtonsDevelopment();
            case ALGAE_BLASTER_KS -> configureAlgaeBlasterKs();
            case PATHPLANNER_TEST -> calibratePathPlanner();
            case CHARACTERIZE_WHEEL_RADIUS -> configureButtonsCharacterizeWheelRadius();
            case CHARACTERIZE_ELEVATOR -> setupSysIdCharacterization(ELEVATOR);
            case CHARACTERIZE_SWERVE_DRIVE_MOTORS -> {
                setupDriving();
                setupSysIdCharacterization(SWERVE);
            }
            case CHARACTERIZE_ALGAE_BLASTER_ARM -> setupSysIdCharacterization(ALGAE_BLASTER);
            case CHARACTERIZE_ALGAE_INTAKE_ARM -> setupSysIdCharacterization(ALGAE_INTAKE);
            case CHARACTERIZE_SWERVE_AZIMUTH -> setupAzimuthCharacterization();
        }
    }

    private static void calibratePathPlanner() {
        setupDriving();
        DRIVER_CONTROLLER.getDPad(Controller.DPad.DOWN)
                .whileTrue((ELEVATOR.runElevatorDownwards()));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.LEFT).whileTrue(ELEVATOR.runCurrentZeroing());
    }

    private static void configureAlgaeBlasterKs() {
        final StaticFrictionCharacterization forwardFrictionCharacterization = new StaticFrictionCharacterization(
                ALGAE_BLASTER, AlgaeBlasterConstants.ARM_BLASTER_MOTOR, false);

        final StaticFrictionCharacterization backwardsFrictionCharacterization = new StaticFrictionCharacterization(
                ALGAE_BLASTER, AlgaeBlasterConstants.ARM_BLASTER_MOTOR, true);

        DRIVER_CONTROLLER.getDPad(Controller.DPad.DOWN)
                .whileTrue(forwardFrictionCharacterization);

        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP)
                .whileTrue(backwardsFrictionCharacterization);

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.VERTICAL));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_OUT));

    }

    private static void characterizeElevatorKSandKG() {
        setupDriving();

        DRIVER_CONTROLLER.getDPad(Controller.DPad.RIGHT).whileTrue(
                ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L1));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP).whileTrue(
                ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L2));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.LEFT).whileTrue(
                ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L3));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.DOWN).whileTrue(
                ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(CORAL_INTAKE.releaseGamePiece());

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(
                Commands.runEnd(() -> ELEVATOR.sysIdDrive(-1.9), ELEVATOR::stop, ELEVATOR));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(
                Commands.runEnd(() -> ELEVATOR.sysIdDrive(1.9), ELEVATOR::stop, ELEVATOR));
    }

    private static void configureButtonsDevelopment() {
        setupDriving();

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.VERTICAL));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_OUT));
    }

    private static void configureButtonsTeleop() {
        setupDriving();

        final Trigger leftBumper = new Trigger(DRIVER_CONTROLLER.getButton(Controller.Inputs.LEFT_BUMPER));
        final Trigger rightBumper = new Trigger(DRIVER_CONTROLLER.getButton(Controller.Inputs.RIGHT_BUMPER));

        leftBumper.and(rightBumper.negate()).whileTrue(CoralManipulationCommands.pathfindToBranchAndScoreForTeleop(PathfindingConstants.Branch.LEFT_BRANCH));
        rightBumper.and(leftBumper.negate()).whileTrue(CoralManipulationCommands.pathfindToBranchAndScoreForTeleop(PathfindingConstants.Branch.RIGHT_BRANCH));

        DRIVER_CONTROLLER.getStick(Controller.Stick.LEFT_STICK).whileTrue(CoralManipulationCommands.eatFromFeeder());
        DRIVER_CONTROLLER.getStick(Controller.Stick.RIGHT_STICK).whileTrue(CoralManipulationCommands.scoreCoralFromCurrentLevelAndBlastAlgaeForTeleop());

        DRIVER_CONTROLLER.getDPad(Controller.DPad.DOWN)
                .whileTrue((ELEVATOR.runElevatorDownwards()));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.LEFT).whileTrue(ELEVATOR.runCurrentZeroing());

        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP)
                .whileTrue(ClimbingCommands.rotateToCage()
                .alongWith(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.CLIMB_INSERT)));

        //TODO: REmove, THIS IS FOR DEUBGGING.
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(PathfindingCommands.pathfindToBranchBezier(PathfindingConstants.Branch.LEFT_BRANCH, FieldConstants.ReefFace.FACE_0));

        setupOperatorKeyboardButtons();
        setupTeleopLEDs();
    }

    private static void configureButtonsCharacterizeWheelRadius() {
        setupDriving();

        final Command wheelRadiusCharacterization = new WheelRadiusCharacterization(
                SWERVE,
                ROBOT_CONFIG.moduleLocations,
                SWERVE::getDriveWheelPositionsRadians,
                () -> SWERVE.getGyroHeading() * 2 * Math.PI,
                (speed) -> SWERVE
                        .driveRobotRelative(new ChassisSpeeds(0, 0, speed), true)
        );

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue((wheelRadiusCharacterization));
    }

    private static void setupSysIdCharacterization(GenericSubsystem subsystem) {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private static void setupOperatorKeyboardButtons() {
        OPERATOR_CONTROLLER.one().onTrue(new InstantCommand(() -> CoralManipulationCommands.CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L1));
        OPERATOR_CONTROLLER.two().onTrue(new InstantCommand(() -> CoralManipulationCommands.CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L2));
        OPERATOR_CONTROLLER.three().onTrue(new InstantCommand(() -> CoralManipulationCommands.CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L3));

        OPERATOR_CONTROLLER.six().onTrue(
                        (new InstantCommand(() -> SHOULD_BLAST_ALGAE = true)));

        OPERATOR_CONTROLLER.six().onFalse(
                ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN)
                        .alongWith(new InstantCommand(() -> SHOULD_BLAST_ALGAE = false))
        );
    }

    private static void setupTeleopLEDs() {
        final Trigger hasCoral = new Trigger(CORAL_INTAKE::hasCoral);
        final Trigger isEndOfMatch = new Trigger(() -> DriverStation.getMatchTime() <= 30);

        hasCoral.onTrue(LEDS.setLEDStatus(Leds.LEDMode.INTAKE_LOADED, 4)
                .alongWith(DRIVER_CONTROLLER.rumble(0.5, 1)));

        hasCoral.onFalse(LEDS.setLEDStatus(Leds.LEDMode.INTAKE_EMPTIED, 2));

        isEndOfMatch.onTrue(LEDS.setLEDStatus(Leds.LEDMode.END_OF_MATCH, 5));
    }

    private static void setupUserButtonDebugging() {
        USER_BUTTON.toggleOnTrue(
                Commands.startEnd(
                                () -> setModeOfAllSubsystems(MotorProperties.IdleMode.COAST),
                                () -> setModeOfAllSubsystems(MotorProperties.IdleMode.BRAKE)
                        ).alongWith(LEDS.setLEDStatus(Leds.LEDMode.DEBUG_MODE, 1500))
                        .andThen(LEDS.setLEDStatus(Leds.LEDMode.DEFAULT, 0))
        ).debounce(1);
    }

    private static void setModeOfAllSubsystems(MotorProperties.IdleMode idleMode) {
        ELEVATOR.setIdleMode(idleMode);
        CORAL_INTAKE.setIdleMode(idleMode);
        ALGAE_BLASTER.setIdleMode(idleMode);
        ALGAE_INTAKE.setIdleMode(idleMode);
    }

    private static void setupAzimuthCharacterization() {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getCurrentPose().rotateBy(Rotation2d.fromDegrees(90))));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getCurrentPose().rotateBy(Rotation2d.fromDegrees(180))));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getCurrentPose().rotateBy(Rotation2d.fromDegrees(270))));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getCurrentPose().rotateBy(Rotation2d.fromDegrees(360))));
    }

    private static void setupDriving() {
        final DoubleSupplier translationSupplier = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
        final DoubleSupplier strafeSupplier = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
        final DoubleSupplier rotationSupplier = () -> -DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X) * 4;

        SWERVE.setDefaultCommand(
                SwerveCommands.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,
                        rotationSupplier,

                        () -> false
                ));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
        DRIVER_CONTROLLER.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
    }
}
