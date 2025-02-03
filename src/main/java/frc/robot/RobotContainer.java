package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.Questionnaire;
import frc.lib.util.flippable.Flippable;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.algaeblaster.AlgaeBlaster;
import frc.robot.subsystems.algaeintake.AlgaeIntake;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.PathPlannerConstants;

import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.*;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            FRONT_LEFT_CAMERA,
            FRONT_RIGHT_CAMERA,
            REAR_LEFT_CAMERA,
            REAR_RIGHT_CAMERA
    );

    public static final Swerve SWERVE = new Swerve();
    public static final Elevator ELEVATOR = new Elevator();
    public static final CoralIntake CORAL_INTAKE = new CoralIntake();
    public static final AlgaeBlaster ALGAE_BLASTER = new AlgaeBlaster();
    public static final AlgaeIntake ALGAE_INTAKE = new AlgaeIntake();
    public static final Leds LEDS = new Leds();

    public static Questionnaire questionnaire;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        Flippable.init();
        PathPlannerConstants.initializePathPlanner();

        setupAutonomous();
        setupLEDs();

        ButtonControls.initializeButtons(ButtonControls.ButtonLayout.TELEOP);
    }

    public Command getAutonomousCommand() {
        return questionnaire.getCommand();
    }

    public String getAutoName() {
        return questionnaire.getSelected();
    }

    private void setupLEDs() {
        LEDS.setDefaultCommand(LEDS.setLEDStatus(Leds.LEDMode.DEFAULT, 0));

        final int LOW_BATTERY_THRESHOLD = 150;
        final int[] lowBatteryCounter = {0};

        final Trigger batteryLowTrigger = new Trigger(() -> {
            if (RobotController.getBatteryVoltage() < 11.7)
                lowBatteryCounter[0]++;

            return LOW_BATTERY_THRESHOLD < lowBatteryCounter[0];
        });

        batteryLowTrigger.onTrue(LEDS.setLEDStatus(Leds.LEDMode.BATTERY_LOW, 5));
    }

    private void setupAutonomous() {
        questionnaire = new Questionnaire();
    }
}