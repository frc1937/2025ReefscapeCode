package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.WheelRadiusCharacterization;
import frc.lib.generic.hardware.KeyboardController;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.Controller;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.AlgaeManipulationCommands;
import frc.robot.commands.ClimbingCommands;
import frc.robot.commands.CoralManipulationCommands;
import frc.robot.subsystems.algaeintake.AlgaeIntakeConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.robot.RobotContainer.*;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class ButtonControls {
    public enum ButtonLayout {
        DEVELOPMENT,
        TELEOP,
        CHARACTERIZE_ELEVATOR,
        CHARACTERIZE_SWERVE_DRIVE_MOTORS,
        CHARACTERIZE_WHEEL_RADIUS,
        CHARACTERIZE_ALGAE_BLASTER_ARM,
        CHARACTERIZE_ALGAE_INTAKE_ARM
    }

    private static final Controller DRIVER_CONTROLLER = new Controller(0);
    private static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    private static final Trigger USER_BUTTON = new Trigger(RobotController::getUserButton);

    public static void initializeButtons(ButtonLayout layout) {
        setupUserButtonDebugging();

        switch (layout) {
            case DEVELOPMENT -> configureButtonsDevelopment();
            case TELEOP -> configureButtonsTeleop();
            case CHARACTERIZE_WHEEL_RADIUS -> configureButtonsCharacterizeWheelRadius();
            case CHARACTERIZE_ELEVATOR -> setupSysIdCharacterization(ELEVATOR);
            case CHARACTERIZE_SWERVE_DRIVE_MOTORS -> setupSysIdCharacterization(SWERVE);
            case CHARACTERIZE_ALGAE_INTAKE_ARM -> setupSysIdCharacterization(ALGAE_INTAKE);
            case CHARACTERIZE_ALGAE_BLASTER_ARM -> setupSysIdCharacterization(ALGAE_BLASTER);
        }
    }

    private static void configureButtonsDevelopment() {
        final DoubleSupplier driveSign = () -> Flippable.isRedAlliance() ? 1 : -1;

        final DoubleSupplier translationSupplier = () -> driveSign.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
        final DoubleSupplier strafeSupplier = () -> driveSign.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
        final DoubleSupplier rotationSupplier = () -> -DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X) * 3;

        SWERVE.setDefaultCommand(
                SwerveCommands.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,
                        rotationSupplier,

                        () -> DRIVER_CONTROLLER.getStick(Controller.Stick.RIGHT_STICK).getAsBoolean()
                ));


        final Trigger leftBumper = new Trigger(DRIVER_CONTROLLER.getButton(Controller.Inputs.LEFT_BUMPER));
        final Trigger rightBumper = new Trigger(DRIVER_CONTROLLER.getButton(Controller.Inputs.RIGHT_BUMPER));

        leftBumper.and(rightBumper.negate()).whileTrue(CoralManipulationCommands.pathfindToLeftBranchAndScore());
        rightBumper.and(leftBumper.negate()).whileTrue(CoralManipulationCommands.pathfindToRightBranchAndScore());

        DRIVER_CONTROLLER.getStick(Controller.Stick.RIGHT_STICK).whileTrue(CoralManipulationCommands.eatFromFeeder());
        DRIVER_CONTROLLER.getStick(Controller.Stick.LEFT_STICK).whileTrue(CoralManipulationCommands.pathfindToFeederAndEat());

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L3));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(ELEVATOR.setTargetHeight(ElevatorConstants.ElevatorHeight.L1));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(ALGAE_INTAKE.setAlgaeIntakeState(AlgaeIntakeConstants.IntakeArmState.EXTENDED));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(ALGAE_INTAKE.setAlgaeIntakeState(AlgaeIntakeConstants.IntakeArmState.RETRACTED));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP).whileTrue(ClimbingCommands.pathfindToCageAndClimb()
                .alongWith(DRIVER_CONTROLLER.rumble(0.3, 3)));

        setupOperatorKeyboardButtons();
        setupTeleopLEDs();
    }

    private static void configureButtonsTeleop() {
        final DoubleSupplier driveSign = () -> Flippable.isRedAlliance() ? 1 : -1;

        final DoubleSupplier translationSupplier = () -> driveSign.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
        final DoubleSupplier strafeSupplier = () -> driveSign.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
        final DoubleSupplier rotationSupplier = () -> -DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X) * 3;

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

        leftBumper.and(rightBumper.negate()).whileTrue(CoralManipulationCommands.pathfindToLeftBranchAndScore());
        rightBumper.and(leftBumper.negate()).whileTrue(CoralManipulationCommands.pathfindToRightBranchAndScore());

        DRIVER_CONTROLLER.getStick(Controller.Stick.RIGHT_STICK).whileTrue(CoralManipulationCommands.eatFromFeeder());
        DRIVER_CONTROLLER.getStick(Controller.Stick.LEFT_STICK).whileTrue(CoralManipulationCommands.pathfindToFeederAndEat());

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(AlgaeManipulationCommands.releaseAlgae());
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(CoralManipulationCommands.scoreCoralNoPositionCheck());
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(AlgaeManipulationCommands.blastAlgaeOffReef());
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(AlgaeManipulationCommands.intakeAlgae());

        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP).whileTrue(ClimbingCommands.pathfindToCageAndClimb()
                .alongWith(DRIVER_CONTROLLER.rumble(0.3, 3)));

        setupOperatorKeyboardButtons();
        setupTeleopLEDs();
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
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private static void setupOperatorKeyboardButtons() {
        OPERATOR_CONTROLLER.one().onTrue(new InstantCommand(() -> CoralManipulationCommands.CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L1));
        OPERATOR_CONTROLLER.two().onTrue(new InstantCommand(() -> CoralManipulationCommands.CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L2));
        OPERATOR_CONTROLLER.three().onTrue(new InstantCommand(() -> CoralManipulationCommands.CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.L3));
        OPERATOR_CONTROLLER.five().onTrue(new InstantCommand(() -> CoralManipulationCommands.CURRENT_SCORING_LEVEL = ElevatorConstants.ElevatorHeight.FEEDER));

        OPERATOR_CONTROLLER.six().whileTrue(ClimbingCommands.climbCageNoAlignment());
        OPERATOR_CONTROLLER.seven().onTrue(new InstantCommand(() -> CoralManipulationCommands.SHOULD_BLAST_ALGAE = true));
    }

    private static void setupTeleopLEDs() {
        final Trigger hasCoral = new Trigger(CORAL_INTAKE::hasCoral);
        final Trigger isEndOfMatch = new Trigger(() -> DriverStation.getMatchTime() <= 15);

        hasCoral.onTrue(LEDS.setLEDStatus(Leds.LEDMode.INTAKE_LOADED, 3)
                .alongWith(DRIVER_CONTROLLER.rumble(0.5, 1)));
        hasCoral.onFalse(LEDS.setLEDStatus(Leds.LEDMode.INTAKE_EMPTIED, 3));

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
}
