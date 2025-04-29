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
import frc.lib.generic.characterization.CameraPositionCharacterization;
import frc.lib.generic.characterization.StaticFrictionCharacterization;
import frc.lib.generic.characterization.WheelRadiusCharacterization;
import frc.lib.generic.hardware.controllers.Controller;
import frc.lib.generic.hardware.controllers.KeyboardController;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.ConveyorCommands;
import frc.robot.commands.pathfinding.PathfindingConstants;
import frc.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_X;
import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_Y;
import static frc.robot.RobotContainer.*;
import static frc.robot.commands.CoralManipulationCommands.*;
import static frc.robot.commands.pathfinding.BranchPathfinding.pathAndScoreWithOverride;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;
import static frc.robot.subsystems.swerve.SwerveCommands.rotateToTarget;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class ButtonControls {
    public enum ButtonLayout {
        DEVELOPMENT,
        ELEVATOR_KS,
        TELEOP,
        ALGAE_BLASTER_KS,
        CHARACTERIZE_ELEVATOR,
        CHARACTERIZE_SWERVE_DRIVE_MOTORS,
        CHARACTERIZE_WHEEL_RADIUS,
        CHARACTERIZE_ALGAE_BLASTER_ARM,
        CHARACTERIZE_SWERVE_AZIMUTH,
        PATHPLANNER_TEST,
        CLIMB_CHARACTERIZATION
    }


    private static final Controller DRIVER_CONTROLLER = new Controller(0);
    private static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    public static final DoubleSupplier DRIVE_SIGN = () -> Flippable.isRedAlliance() ? 1 : -1;

    private static final DoubleSupplier X_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
    private static final DoubleSupplier Y_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
    private static final DoubleSupplier ROTATION_SUPPLIER = () -> -DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X) * 8;

    private static final Trigger USER_BUTTON = new Trigger(RobotController::getUserButton);

    public static void initializeButtons(ButtonLayout layout) {
        setupUserButtonDebugging();

        switch (layout) {
            case TELEOP -> configureButtonsTeleop();
            case CLIMB_CHARACTERIZATION -> configureClimbCharacterization();
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
            case CHARACTERIZE_SWERVE_AZIMUTH -> setupAzimuthCharacterization();
        }
    }

    private static void configureClimbCharacterization() {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(CLIMB.runVoltage(-4));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(CLIMB.runVoltage(4));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.DOWN).whileTrue(CLIMB.setClimbState(ClimbConstants.ClimbState.INITIAL_STATE));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP).whileTrue(CLIMB.setClimbState(ClimbConstants.ClimbState.READY_CLIMB));
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

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(ALGAE_BLASTER.setArmTargetState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(ALGAE_BLASTER.setArmTargetState(AlgaeBlasterConstants.BlasterArmState.VERTICAL));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(ALGAE_BLASTER.setArmTargetState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_OUT));

    }

    private static void characterizeElevatorKSandKG() {
        setupDriving();

        DRIVER_CONTROLLER.getDPad(Controller.DPad.RIGHT).whileTrue(
                ELEVATOR.setTargetHeight(L1).andThen(
                        ELEVATOR.maintainPosition()
                ));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP).whileTrue(
                ELEVATOR.setTargetHeight(L2).andThen(
                        ELEVATOR.maintainPosition()
                ));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.LEFT).whileTrue(
                ELEVATOR.setTargetHeight(L3).andThen(
                        ELEVATOR.maintainPosition()
                ));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.DOWN).whileTrue(
                ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER).andThen(
                        ELEVATOR.maintainPosition()
                ));
    }

    private static void configureButtonsDevelopment() {
        setupDriving();

        final CameraPositionCharacterization frontLeft = new CameraPositionCharacterization(
                AprilTagCameraConstants.FRONT_LEFT_CAMERA::getEstimatedRobotPose,
                AprilTagCameraConstants.ROBOT_TO_FRONT_LEFT_CAMERA.getRotation().toRotation2d(),
                (speed) -> SWERVE.driveRobotRelative(0, 0, speed, false),
                SWERVE
        );

        final CameraPositionCharacterization frontRight = new CameraPositionCharacterization(
                AprilTagCameraConstants.FRONT_RIGHT_CAMERA::getEstimatedRobotPose,
                AprilTagCameraConstants.ROBOT_TO_FRONT_RIGHT_CAMERA.getRotation().toRotation2d(),
                (speed) -> SWERVE.driveRobotRelative(0, 0, speed, false),
                SWERVE
        );

        final CameraPositionCharacterization rearLeft = new CameraPositionCharacterization(
                AprilTagCameraConstants.REAR_LEFT_CAMERA::getEstimatedRobotPose,
                AprilTagCameraConstants.ROBOT_TO_REAR_LEFT_CAMERA.getRotation().toRotation2d(),
                (speed) -> SWERVE.driveRobotRelative(0, 0, speed, false),
                SWERVE
        );

        final CameraPositionCharacterization rearRight = new CameraPositionCharacterization(
                AprilTagCameraConstants.REAR_RIGHT_CAMERA::getEstimatedRobotPose,
                AprilTagCameraConstants.ROBOT_TO_REAR_RIGHT_CAMERA.getRotation().toRotation2d(),
                (speed) -> SWERVE.driveRobotRelative(0, 0, speed, false),
                SWERVE
        );

        final Command wheelRadiusCharacterization = new WheelRadiusCharacterization(
                SWERVE,
                ROBOT_CONFIG.moduleLocations,
                SWERVE::getDriveWheelPositionsRadians,
                () -> SWERVE.getGyroHeading() * 2 * Math.PI,
                (speed) -> SWERVE
                        .driveRobotRelative(new ChassisSpeeds(0, 0, speed), true)
        );

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue((wheelRadiusCharacterization));

//        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(frontLeft);
//        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(frontRight);
//        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(rearRight);
//        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(rearLeft);
    }

    private static void configureButtonsTeleop() {
        setupDriving();

        ALGAE_BLASTER.setDefaultCommand(
                ALGAE_BLASTER.setArmStateContinuous(AlgaeBlasterConstants.BlasterArmState.DEFAULT_POSE)
        );

        final Trigger isJoystickStill = new Trigger(() ->
                   Math.abs(Y_SUPPLIER.getAsDouble()) <= 0.04
                && Math.abs(X_SUPPLIER.getAsDouble()) <= 0.04);

        final Trigger leftBumper = DRIVER_CONTROLLER.getButton(Controller.Inputs.LEFT_BUMPER);
        final Trigger rightBranch = DRIVER_CONTROLLER.getButton(Controller.Inputs.RIGHT_BUMPER);

        leftBumper.toggleOnTrue(
                LEDS.setLEDStatus(Leds.LEDMode.AUTOMATION, 100).asProxy()
                        .withDeadline(
                            pathAndScoreWithOverride(PathfindingConstants.Branch.LEFT_BRANCH,
                            X_SUPPLIER, Y_SUPPLIER, ROTATION_SUPPLIER,
                            isJoystickStill.negate()))
        );

        rightBranch.toggleOnTrue(
                LEDS.setLEDStatus(Leds.LEDMode.AUTOMATION, 100).asProxy()
                        .withDeadline(
                            pathAndScoreWithOverride(PathfindingConstants.Branch.RIGHT_BRANCH,
                            X_SUPPLIER, Y_SUPPLIER, ROTATION_SUPPLIER,
                            isJoystickStill.negate()))
        );

        DRIVER_CONTROLLER.getStick(Controller.Stick.LEFT_STICK).whileTrue(eatFromFeeder());
        DRIVER_CONTROLLER.getStick(Controller.Stick.RIGHT_STICK).whileTrue(justReleaseACoralTeleop());

        DRIVER_CONTROLLER.getDPad(Controller.DPad.DOWN).whileTrue(CLIMB.runVoltage(-12));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP).whileTrue(CLIMB.runVoltage(12));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(ConveyorCommands.scoreToL4(PathfindingConstants.Branch.LEFT_BRANCH));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(yeetAlgaeWithAlignment());

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(ConveyorCommands.moveFromIntakeToL4());

        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(SwerveCommands.driveToCoral());

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
        OPERATOR_CONTROLLER.one().onTrue(new InstantCommand(() -> CURRENT_SCORING_LEVEL = L1).ignoringDisable(true));
        OPERATOR_CONTROLLER.two().onTrue(new InstantCommand(() -> CURRENT_SCORING_LEVEL = L2).ignoringDisable(true));
        OPERATOR_CONTROLLER.three().onTrue(new InstantCommand(() -> CURRENT_SCORING_LEVEL = L3).ignoringDisable(true));
        OPERATOR_CONTROLLER.four().onTrue(new InstantCommand(() -> CURRENT_SCORING_LEVEL = L4).ignoringDisable(true)); //todo: test

        OPERATOR_CONTROLLER.five().whileTrue(CORAL_INTAKE.setMotorVoltage(-2));

        OPERATOR_CONTROLLER.seven()
                .whileTrue(CLIMB.runVoltage(12));
//        OPERATOR_CONTROLLER.six().onTrue(
//                        (new InstantCommand(() -> SHOULD_BLAST_ALGAE = true)));
//
//        OPERATOR_CONTROLLER.six()
//                .onFalse(
//                        ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN)
//                        .alongWith(new InstantCommand(() -> SHOULD_BLAST_ALGAE = false))
//        );
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
        SWERVE.setDefaultCommand(
                SwerveCommands.driveOpenLoop(
                        X_SUPPLIER,
                        Y_SUPPLIER,
                        ROTATION_SUPPLIER,

                        () -> false
                ));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
        DRIVER_CONTROLLER.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
    }
}
