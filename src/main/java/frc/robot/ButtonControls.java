package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.WheelRadiusCharacterization;
import frc.lib.generic.hardware.KeyboardController;
import frc.lib.util.Controller;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.GamepieceManipulationCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.robot.RobotContainer.ELEVATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class ButtonControls {
    public enum ButtonLayout {
        TELEOP,
        CHARACTERIZE_ELEVATOR,
        CHARACTERIZE_SWERVE_DRIVE_MOTORS,
        CHARACTERIZE_WHEEL_RADIUS,
        CHARACTERIZE_ALGAE_OUTTAKE_ARM,
        CHARACTERIZE_ALGAE_INTAKE_ARM
    }

    private static final Controller DRIVER_CONTROLLER = new Controller(0);
    private static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    private static final Trigger USER_BUTTON = new Trigger(RobotController::getUserButton);

    public static void initializeButtons(ButtonLayout layout) {
        switch (layout) {
            case TELEOP -> configureButtonsTeleop();
            case CHARACTERIZE_WHEEL_RADIUS -> configureButtonsCharacterizeWheelRadius();
            case CHARACTERIZE_ELEVATOR -> setupSysIdCharacterization(ELEVATOR);
            case CHARACTERIZE_SWERVE_DRIVE_MOTORS -> setupSysIdCharacterization(SWERVE);
        }
    }

    private static void configureButtonsTeleop() {
        DoubleSupplier driveSign = () -> Flippable.isRedAlliance() ? 1 : -1;

        DoubleSupplier translationSupplier = () -> driveSign.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> driveSign.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
        DoubleSupplier rotationSupplier = () -> -DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X) * 3;

        SWERVE.setDefaultCommand(
                SwerveCommands.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,
                        rotationSupplier,

                        () -> DRIVER_CONTROLLER.getStick(Controller.Stick.RIGHT_STICK).getAsBoolean()
                ));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
        DRIVER_CONTROLLER.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());

        final Trigger leftBumper = new Trigger(DRIVER_CONTROLLER.getButton(Controller.Inputs.LEFT_BUMPER));
        final Trigger rightBumper = new Trigger(DRIVER_CONTROLLER.getButton(Controller.Inputs.RIGHT_BUMPER));

        leftBumper.and(rightBumper.negate()).whileTrue(GamepieceManipulationCommands.pathfindToLeftBranchAndScore());
        rightBumper.and(leftBumper.negate()).whileTrue(GamepieceManipulationCommands.pathfindToRightBranchAndScore());

        DRIVER_CONTROLLER.getStick(Controller.Stick.LEFT_STICK).whileTrue(GamepieceManipulationCommands.pathfindToFeederAndEat());

        setupOperatorKeyboardButtons();
    }

    private static void configureButtonsCharacterizeWheelRadius() {
        final Command wheelRadiusCharacterization = new WheelRadiusCharacterization(
                SWERVE,
                ROBOT_CONFIG.moduleLocations,
                SWERVE::getDriveWheelPositionsRadians,
                () -> SWERVE.getGyroHeading() * 2 * Math.PI,
                SWERVE::runDriveMotorWheelCharacterization
        );

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(
                SwerveCommands.driveOpenLoop(() -> 0, () -> 0, () -> 0.1, () -> true).withTimeout(0.2)
                        .andThen(wheelRadiusCharacterization));
    }

    private static void setupSysIdCharacterization(GenericSubsystem subsystem) {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private static void setupOperatorKeyboardButtons() {
        OPERATOR_CONTROLLER.one().onTrue(new InstantCommand(() -> GamepieceManipulationCommands.currentScoringLevel = ElevatorConstants.ElevatorHeight.L1));
        OPERATOR_CONTROLLER.two().onTrue(new InstantCommand(() -> GamepieceManipulationCommands.currentScoringLevel = ElevatorConstants.ElevatorHeight.L2));
        OPERATOR_CONTROLLER.three().onTrue(new InstantCommand(() -> GamepieceManipulationCommands.currentScoringLevel = ElevatorConstants.ElevatorHeight.L3));

        //TODO:
        //t, y: upper faces
        // f, j: left, right faces
        // v, b: lower faces
        //g, h: LEFT RIGHT brnahces
    }
}
