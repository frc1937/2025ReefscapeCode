package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static java.lang.Math.round;

class ButtonTest {
    private double shooterLength = 2;

    private void notWorkingMethod() {
//        double rpm = Conversions.RPMFromTangentialVelocity(MetersPerSecond.of(2), Inches.of(4));
//
//        System.out.println(rpm);
//
        Map<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();

        for (AprilTag aprilTag : AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTags())
            TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);

        for (Pose3d tagPosition : TAG_ID_TO_POSE.values()) {
            System.out.println("Y:" + TAG_ID_TO_POSE.get(tagPosition).getY()); //Y:5.547868
            System.out.println("X:" + TAG_ID_TO_POSE.get(tagPosition).getX()); //X:-0.0381
        }
//
    }

    @Test
    void testButton() {
        DriverStation.silenceJoystickConnectionWarning(true);
//        notWorkingMethod();
//
//        Random random = new Random();
//        TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
//
//        for (int i = 0; i < 100; i++) {
//            setpoint = testTrapezoidalPIDController((int) Math.pow(i, 2), setpoint);//* random.nextInt(10));
//        }
    }

    private void testProfiledPIDController(int additionToCurrentAngle) {
        final ProfiledPIDController profiledPIDController = new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(1, 1));

        Rotation2d currentAngle = Rotation2d.fromDegrees(20 + additionToCurrentAngle);
        Rotation2d targetAngle = Rotation2d.fromDegrees(70);

        profiledPIDController.reset(currentAngle.getRadians());
        profiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        double radsPerSec = profiledPIDController.calculate(currentAngle.getRadians(), targetAngle.getRadians());

        System.out.println("RadsPerSec: " + radsPerSec);
    }

    private void testPIDController(int doubleToAddToCurrent) {
        PIDController controller = new PIDController(1, 0, 0);

        controller.enableContinuousInput(-180, 180);

        Rotation2d currentAngle = Rotation2d.fromDegrees(20 + doubleToAddToCurrent);
        Rotation2d targetAngle = Rotation2d.fromDegrees(70);

        double degreesPerSecond = toNDecimals(controller.calculate(currentAngle.getDegrees(), targetAngle.getDegrees()), 4);
        double radiansPerSecond = toNDecimals(controller.calculate(currentAngle.getRadians(), targetAngle.getRadians()), 4);
        double distanceFromSetpoint = targetAngle.minus(currentAngle).getDegrees();

        System.out.println("NEW - POINT!");
        System.out.println("Distance from setpoint [DEGS]: " + distanceFromSetpoint);
        System.out.println("DegreesPerSecond [DEGS P S]: " + degreesPerSecond);
        System.out.println("RadiansPerSecond [RADS P S]: " + radiansPerSecond);

        System.out.println("(" + round(distanceFromSetpoint) + ", " + round(degreesPerSecond) + ")");
    }

    private double toNDecimals(double input, int n) {
        return Math.floor(input * Math.pow(10, n)) / Math.pow(10, n);
    }

    private TrapezoidProfile.State testTrapezoidalPIDController(int additionToCurrentAngle, TrapezoidProfile.State setpoint) {
        final PIDController controller = new PIDController(1, 0, 0);

        final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.5, 1);
        final TrapezoidProfile.State goal = new TrapezoidProfile.State(50, 0);

        TrapezoidProfile profile = new TrapezoidProfile(constraints);

        setpoint = profile.calculate(0.02, setpoint, goal);

//        System.out.println("Setpoint loc: " + setpoint.position);
//        System.out.println("Erm, what the result? " + controller.calculate(additionToCurrentAngle, setpoint.position));

        System.out.println("(" + additionToCurrentAngle + ", " + controller.calculate(additionToCurrentAngle, setpoint.position)+ ")");

        return setpoint;
    }

    private void testHolonomicProfiledPIDController(int doubleToAddToCurrent) {
        final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.5, 1);

        final HolonomicDriveController driveController =
                new HolonomicDriveController(
                        new PIDController(0, 0, 0),
                        new PIDController(0, 0, 0),
                        new ProfiledPIDController(50000, 0, 0, constraints)
                );

        driveController.getThetaController().enableContinuousInput(-180, 180);
        driveController.setTolerance(new Pose2d(0.2, 0.2, Rotation2d.fromDegrees(1.2)));

        Rotation2d currentAngle = Rotation2d.fromDegrees(20 + doubleToAddToCurrent);
        Rotation2d targetAngle = Rotation2d.fromDegrees(70);

        ChassisSpeeds chassisSpeeds = driveController.calculate(
                new Pose2d(0, 0, currentAngle),
                new Pose2d(0, 0, targetAngle),
                0,
                targetAngle
        );

        double omegaRads = round(chassisSpeeds.omegaRadiansPerSecond);
        double difference = round((targetAngle.minus(currentAngle).getDegrees()));

//        System.out.println("Another iteration of PID");
//        System.out.println("Current angle: " + currentAngle.getDegrees() + ", Target angle: " + targetAngle.getDegrees() +
//                ", Difference: " + difference);
//        System.out.println("Omega rads: " + omegaRads);
        System.out.println("(" + difference + ", " + omegaRads + ")");
    }

    private void testWhetherTheShooterThingWorks() {
        int[] testValuesYaw = {0, 90, 180, 270};
        int[] testValuesPitch = {0, 20, 40, 90};

        for (int i = 0; i < 4; i++) {
            Pose2d robotPose = new Pose2d(100, 5, Rotation2d.fromDegrees(testValuesYaw[i]));
            for (int j = 0; j < 4; j++) {
                double targetPitchAngle = testValuesPitch[j];

                System.out.println("Initial parameters:");
                System.out.println("Robot (x, y, azimuth): " + robotPose.getX() + ", " + robotPose.getY() + ", " + robotPose.getRotation().getDegrees());
                System.out.println("Pitch angle: " + targetPitchAngle + "\n");

                Pose3d noteExitPos = calculateShooterEndEffectorFieldRelativePose(robotPose, targetPitchAngle);
                Pose3d noteExitPose2 = getNoteExitPoseRobodox599(robotPose, targetPitchAngle);

                printPose3dNicely(noteExitPos, "verifiedExitPos");
                System.out.println();
                printPose3dNicely(noteExitPose2, "elysiumExitPose");

                Assertions.assertEquals(noteExitPos, noteExitPose2);
            }
        }

//        System.out.println("The max speed of swerve calculated: " + MAX_SPEED_MPS);
    }

    private void printPose3dNicely(Pose3d pose, String name) {
        System.out.println("Pose3d " + name + ":");
        System.out.println("(" + pose.getX() + ", " + pose.getY() + ", " + pose.getZ() + ")"
                + ", ROLL" + toDegrees(pose.getRotation().getX())
                + ", PITCH" + toDegrees(pose.getRotation().getY())
                + ", YAW " + toDegrees(pose.getRotation().getZ()));
    }

    private double toDegrees(double radians) {
        return round(radians * 180 / Math.PI);
    }
//    private Pose3d getNoteExitPoseTEST(Pose2d robotPose, double targetAngle) {
//        Rotation2d pitchAngle = Rotation2d.fromDegrees(targetAngle);
//        Pose3d robotPose3d = new Pose3d(robotPose);
//
//        Transform3d robotToPivot = new Transform3d( //todo: Tune from cad
//                -0.2,
//                0,
//                0.3,
//                new Rotation3d(0, 0, 0)
//        );
//
//        Pose3d pivotPose = robotPose3d.transformBy(robotToPivot);
//
//        double pitchLength = 0.6; //todo: Tune from cad
//
//        Transform3d pivotToShooterEnd = new Transform3d(
//                pitchLength,
//                0,
//                0,
//                new Rotation3d(0, -pitchAngle.getRadians(), 0)
//        );
//
//        Pose3d noteExitPos = pivotPose.plus(pivotToShooterEnd);
//
//        return noteExitPos;
//    }

    private Pose3d getNoteExitPoseRobodox599(Pose2d robotPose, double targetAngle) {
        Rotation2d azimuthAngleToSpeaker = robotPose.getRotation();
        Rotation2d pitchAngle = Rotation2d.fromDegrees(targetAngle);

        Pose3d robotPose3d = new Pose3d(new Pose2d(robotPose.getTranslation(), azimuthAngleToSpeaker));

        double PIVOT_POINT_Z_OFFSET = 0.5;
        double PIVOT_POINT_X_OFFSET = -1;

        Transform3d robotToPivot = new Transform3d(
                PIVOT_POINT_X_OFFSET, 0, PIVOT_POINT_Z_OFFSET,
                new Rotation3d(0, -pitchAngle.getRadians(), 0)
        );

        Transform3d pivotToEndEffector = new Transform3d(shooterLength, 0, 0, new Rotation3d());

        Pose3d shooterEndPose = new Pose3d().transformBy(robotToPivot).plus(pivotToEndEffector);
        Transform3d robotToShooterEnd = shooterEndPose.minus(new Pose3d());

        return robotPose3d.transformBy(robotToShooterEnd);
    }

    private Pose3d calculateShooterEndEffectorFieldRelativePose(Pose2d robotPose, double targetAngle) {
        Rotation2d azimuthAngleToSpeaker = robotPose.getRotation();
        Rotation2d pitchAngle = Rotation2d.fromDegrees(targetAngle);

        Pose3d ROBOT_RELATIVE_PIVOT_POINT = new Pose3d(-1, 0, 0.5, new Rotation3d(0, 0, 0));

        Pose3d pivotPoint = ROBOT_RELATIVE_PIVOT_POINT
                .transformBy(
                        new Transform3d(new Translation3d(),
                                new Rotation3d(0, -pitchAngle.getRadians(), 0)));

        Transform3d pivotToEndEffector = new Transform3d(shooterLength, 0, 0, new Rotation3d());

        Pose3d endEffectorSelfRelativePose = pivotPoint.plus(pivotToEndEffector);
        Transform3d robotToEndEffector = endEffectorSelfRelativePose.minus(new Pose3d());
        Pose3d predictedPose = new Pose3d(new Pose2d(robotPose.getTranslation(), azimuthAngleToSpeaker));

        return predictedPose.transformBy(robotToEndEffector);
    }
}

//TODO:
// * Go over all swerve constants. Also fix the Swerve FF. It's not even in the correct units lmfao.
// * Get value from NT (Photonvision) instead of classes ,cause it runs faster than periodic.
//TODO
// * Adding on the above, look into a robot state. Everything feeds data to it, and it interpolates to account for latency.
// * Link for above (https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2022-build-thread/398645/106)
// * Also, (https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/RobotState.java)
// * Great simulations tutorial. (https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2022-build-thread/398645/109?u=wihy)
//TODO
// * Log Match Time amount - perhaps you could make longer autons! (Latency cause of FMS, might add around 0.3s for each mode.)

//TODO
// * Look into trying state machines. Everything has CONSTANT states, and the system will try to achieve them. More reading required.
// * attempt to move the camera to the end of the shooter, AKA non-constant translations. Cheers!
// * When only 1 controller is connected, combine from Operator and Driver. When two, split to their desired controllers.
// * Align with tag command. Should be very simp le.
// * Gyro fallback - use odom velocities instead.

//TODO
// * Look into traj generation.
// * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2023-build-thread/420691/179?u=wihy
// * ^ Coolest auton managing I've ever seen. Look into implementing something similar. Instead of having rigid routines, have setpoints to get to and performs actions at.
// * read when bored: https://docs.wpilib.org/en/stable/docs/software/telemetry/index.html might have useful info.
// * End of MATCH LEDs flashing would be SOOO useful.
// * A better LED system. Please!

// DONE:
// * Wheel radius characterizations seem interesting. look into them
// * TunableNumber impl
// * If battery is low: change LEDs colours.
// * Follow tag command https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/commands/FollowDemoTag.java
