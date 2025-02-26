package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.PID;
import frc.lib.generic.ProfiledPID;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonConfiguration;
import frc.lib.generic.hardware.pigeon.PigeonFactory;
import frc.lib.generic.hardware.pigeon.PigeonSignal;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.GlobalConstants.IS_SIMULATION;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;
import static frc.robot.utilities.PortsConstants.SwervePorts.GYRO_PORT;

public class SwerveConstants {
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(ROBOT_CONFIG.moduleLocations);
    public static final double MAX_SPEED_MPS = 5;

    protected static final SysIdRoutine.Config SYSID_DRIVE_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(7)
    );

    protected static final double
            STEER_GEAR_RATIO = (150.0 / 7.0),
            DRIVE_GEAR_RATIO = (6.75),
            MAX_ROTATION_RAD_PER_S = 3 * Math.PI,
            WHEEL_DIAMETER = ROBOT_CONFIG.moduleConfig.wheelRadiusMeters * 2;

    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.15,
            ROTATION_NEUTRAL_DEADBAND = 0.15;

    private static final TrapezoidProfile.Constraints TRANSLATIONAL_PROFILES_CONSTRAINTS = IS_SIMULATION
            ? new TrapezoidProfile.Constraints(3, 3)
            : new TrapezoidProfile.Constraints(3, 1.2);

    private static final PIDConstants TRANSLATIONAL_PROFILES_CONSTANTS = IS_SIMULATION
            ? new PIDConstants(1.1, 0, 0)
            : new PIDConstants(1.5958, 0, 0.007);

    protected static final ProfiledPID PROFILED_TRANSLATION_CONTROLLER = new ProfiledPID(TRANSLATIONAL_PROFILES_CONSTANTS, 0, TRANSLATIONAL_PROFILES_CONSTRAINTS);
    protected static final ProfiledPID PROFILED_STRAFE_CONTROLLER = new ProfiledPID(TRANSLATIONAL_PROFILES_CONSTANTS, 0, TRANSLATIONAL_PROFILES_CONSTRAINTS);

    protected static final PID PID_TRANSLATION_CONTROLLER = IS_SIMULATION
            ? new PID(1.2, 0, 0, 0.001)
            : new PID(1.45, 0, 0, 0);

    protected static final ProfiledPID SWERVE_ROTATION_CONTROLLER = IS_SIMULATION
            ? new ProfiledPID(0.2, 0, 0,0, new TrapezoidProfile.Constraints(360, 360))
            : new ProfiledPID(0.05, 0, 0, new TrapezoidProfile.Constraints(360, 360));

    //Different rotational controller for long distances.
    protected static final ProfiledPID SWERVE_ROTATIONAL_CONTROLLER_ACCURATE =
            new ProfiledPID(0.2, 0, 0,0, new TrapezoidProfile.Constraints(360, 360));

    protected static final Pigeon GYRO = PigeonFactory.createPigeon2("GYRO", GYRO_PORT);

    public static double yawOffset = 0;

    static {
        configureGyro();
        configureRotationController();
    }

    private static void configureGyro() {
        PigeonConfiguration configuration = new PigeonConfiguration();

        yawOffset = -89.64400482177734;

        configuration.mountPoseYawDegrees = yawOffset;
        configuration.mountPoseRollDegrees = -0.5925159454345703;
        configuration.mountPosePitchDegrees = 0.8338062763214111;

        GYRO.configurePigeon(configuration);

        GYRO.setupSignalUpdates(PigeonSignal.YAW, true);
    }

    private static void configureRotationController() {
        SWERVE_ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
        SWERVE_ROTATION_CONTROLLER.setTolerance(1);

        SWERVE_ROTATIONAL_CONTROLLER_ACCURATE.enableContinuousInput(-180, 180);
        SWERVE_ROTATIONAL_CONTROLLER_ACCURATE.setTolerance(1);

        PROFILED_TRANSLATION_CONTROLLER.setTolerance(0.08);
        PROFILED_STRAFE_CONTROLLER.setTolerance(0.08);
    }
}
