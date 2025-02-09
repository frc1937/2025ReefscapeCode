package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.PID;
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

    public static final double PID_PATHFIND_ACCURACY_THRESHOLD = 0.07;

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

    protected static final PID SWERVE_TRANSLATION_CONTROLLER = IS_SIMULATION
            ? new PID(4, 0, 0)
            : new PID(5, 0, 0);


    protected static final ProfiledPIDController SWERVE_ROTATION_CONTROLLER = IS_SIMULATION ?
            new ProfiledPIDController(
                    0.1, 0, 0,
                    new TrapezoidProfile.Constraints(360, 360)
            )
            :
            new ProfiledPIDController(
                    3.9, 0, 0.05,
                    new TrapezoidProfile.Constraints(360, 360)
            );

    protected static final Pigeon GYRO = PigeonFactory.createIMU("GYRO", GYRO_PORT);

    static {
        configureGyro();
        configureRotationController();
    }

    private static void configureGyro() {
        GYRO.configurePigeon(new PigeonConfiguration());
        GYRO.setupSignalUpdates(PigeonSignal.YAW, true);
    }

    private static void configureRotationController() {
        SWERVE_ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
        SWERVE_ROTATION_CONTROLLER.setTolerance(1);
    }
}
