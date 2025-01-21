package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.util.Controller;
import frc.lib.util.flippable.Flippable;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.*;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.PathPlannerConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.robot.commands.PathfindingCommands.setupFeederPathfinding;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.*;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            FRONT_LEFT_CAMERA,
            FRONT_RIGHT_CAMERA,
            REAR_LEFT_CAMERA,
            REAR_RIGHT_CAMERA
    );

    public static final Swerve SWERVE = new Swerve();
    public static final Leds LEDS = new Leds();
    public static final Elevator ELEVATOR = new Elevator();

    private final Trigger userButton = new Trigger(RobotController::getUserButton);

    private final Controller driveController = new Controller(0);

    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        Flippable.init();
        PathPlannerConstants.initializePathPlanner();

        setupAutonomous();
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        setupLEDs();

        setupFeederPathfinding(driveController.getButton(Controller.Inputs.A));

        driveController.getButton(Controller.Inputs.B).whileTrue(ELEVATOR.setTargetPosition(ElevatorHeight.CLIMB));
        driveController.getButton(Controller.Inputs.A).whileTrue(ELEVATOR.setTargetPosition(ElevatorHeight.L1));
        driveController.getButton(Controller.Inputs.Y).whileTrue(ELEVATOR.setTargetPosition(ElevatorHeight.L3));
        driveController.getButton(Controller.Inputs.X).whileTrue(ELEVATOR.setTargetPosition(ElevatorHeight.FEEDER));

        ELEVATOR.setDefaultCommand(
                ELEVATOR.setTargetPosition(ElevatorHeight.L2)
        );

        configureButtons(ButtonLayout.TELEOP);
    }

    private void configureButtons(ButtonLayout layout) {
        switch (layout) {
            case TELEOP -> configureButtonsTeleop();
        }
    }

    private void setupCharacterization(GenericSubsystem subsystem) {
        driveController.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        driveController.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void setupLEDs() {
        LEDS.setDefaultCommand(LEDS.setLEDStatus(Leds.LEDMode.DEFAULT, 0));

        final int LOW_BATTERY_THRESHOLD = 150;
        final int[] lowBatteryCounter = {0};

        new Trigger(() -> {
            if (RobotController.getBatteryVoltage() < 11.7) lowBatteryCounter[0]++;
            return LOW_BATTERY_THRESHOLD < lowBatteryCounter[0];
        }).onTrue(LEDS.setLEDStatus(Leds.LEDMode.BATTERY_LOW, 5));
    }

    private void setupDriving(DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier) {
        SWERVE.setDefaultCommand(
                SwerveCommands.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,

                        () -> -driveController.getRawAxis(Controller.Axis.RIGHT_X) * 6,
                        () -> driveController.getStick(Controller.Stick.RIGHT_STICK).getAsBoolean()
                ));

        driveController.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
        driveController.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
    }

    private void configureButtonsTeleop() {
        DoubleSupplier translationSupplier = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> -driveController.getRawAxis(LEFT_X);

        setupDriving(translationSupplier, strafeSupplier);

//        driveController.getButton(Controller.Inputs.A)
//            .whileTrue(SwerveCommands.driveAndRotateToClosestNote(translationSupplier, strafeSupplier));


//        driveController.getButton(Controller.Inputs.B).whileTrue(
//                new WheelRadiusCharacterization(
//                        SWERVE,
//                        MODULE_LOCATIONS,
//                        SWERVE::getDriveWheelPositionsRadians,
//                        () -> SWERVE.getGyroHeading().getRadians(),
//                        SWERVE::runWheelCharacterization
//                ));
//        );
    }

    private void setupAutonomous() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser(""));
    }

    private enum ButtonLayout {
        TELEOP
    }
}
