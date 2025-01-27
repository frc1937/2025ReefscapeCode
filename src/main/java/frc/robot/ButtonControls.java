package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.WheelRadiusCharacterization;
import frc.lib.generic.hardware.KeyboardController;
import frc.lib.util.Controller;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.PathfindingCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.robot.RobotContainer.ELEVATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.commands.PathfindingCommands.pathfindToFeeder;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class ButtonControls {
    public enum ButtonLayout {
        TELEOP,
        CHARACTERIZE_ELEVATOR,
        CHARACTERIZE_SWERVE,
        CHARACTERIZE_WHEEL_RADIUS,
        CHARACTERIZE_ARM
    }

    private static final Controller DRIVER_CONTROLLER = new Controller(0);
    private static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    private static final Trigger USER_BUTTON = new Trigger(RobotController::getUserButton);

    public static void initializeButtons(ButtonLayout layout) {
        switch (layout) {
            case TELEOP -> configureButtonsTeleop();
            case CHARACTERIZE_WHEEL_RADIUS -> configureButtonsCharacterizeWheelRadius();
        }
    }

    private static void configureButtonsTeleop() {
        DoubleSupplier driveSign = () -> Flippable.isRedAlliance() ? 1 : -1;

        DoubleSupplier translationSupplier = () -> driveSign.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> driveSign.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
        DoubleSupplier rotationSupplier = () -> driveSign.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X);

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

        leftBumper.and(rightBumper.negate()).whileTrue(
                PathfindingCommands.pathfindToLeftBranch().alongWith(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L2))
        );

        rightBumper.and(leftBumper.negate()).whileTrue(
                PathfindingCommands.pathfindToRightBranch().alongWith(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L2))
        );

        rightBumper.and(leftBumper).whileTrue(
                PathfindingCommands.pathfindToFeeder().alongWith(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.FEEDER))
        );

        DRIVER_CONTROLLER.getStick(Controller.Stick.LEFT_STICK).whileTrue(
                ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L1)
        );

        DRIVER_CONTROLLER.getStick(Controller.Stick.RIGHT_STICK).whileTrue(
                ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L3)
        );
    }


    private static void configureButtonsCharacterizeWheelRadius() {
        final Command wheelRadiusCharacterization = new WheelRadiusCharacterization(
                SWERVE,
                ROBOT_CONFIG.moduleLocations,
                SWERVE::getDriveWheelPositionsRadians,
                () -> SWERVE.getGyroHeading() * 2 * Math.PI,
                SWERVE::runDriveMotorWheelCharacterization
        );

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(wheelRadiusCharacterization);
    }

    private void setupSysIdCharacterization(GenericSubsystem subsystem) {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
}
