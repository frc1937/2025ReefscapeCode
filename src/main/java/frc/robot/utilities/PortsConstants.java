package frc.robot.utilities;

/**
 * Class containing constants for ports on the robot.
 * This is useful for keeping track of which ports are used, so no port is used twice.
 */
public class PortsConstants {
    public static int LEDSTRIP_PORT_PWM = 9;

    public static class SwervePorts {
        public static final int FL_STEER_MOTOR_PORT = 10;
        public static final int FR_STEER_MOTOR_PORT = 11;
        public static final int RL_STEER_MOTOR_PORT = 9;
        public static final int RR_STEER_MOTOR_PORT = 12;

        public static final int FR_DRIVE_MOTOR_PORT = 1;
        public static final int FL_DRIVE_MOTOR_PORT = 2;
        public static final int RL_DRIVE_MOTOR_PORT = 3;
        public static final int RR_DRIVE_MOTOR_PORT = 4;

        public static final int FR_STEER_ENCODER_PORT = 1;
        public static final int FL_STEER_ENCODER_PORT = 2;
        public static final int RL_STEER_ENCODER_PORT = 3;
        public static final int RR_STEER_ENCODER_PORT = 4;

        public static final int GYRO_PORT = 30;
    }

    public static class IntakePorts {
        public static final int INTAKE_MOTOR_PORT = 5;
        public static final int BEAM_BREAK_DIO_PORT = 0;
    }

    public static class AlgaePorts {
        public static final int ALGAE_BLASTER_MOTOR_PORT = 16;
        public static final int ALGAE_INTAKE_MOTOR_PORT = 17;
        public static final int ALGAE_ARM_MOTOR_PORT = 18;
    }

    public static class ElevatorPorts {
        public static final int
                MASTER_MOTOR_PORT = 28,
                SLAVE_MOTOR_PORT = 15,
                TOP_BEAM_BREAK_DIO_PORT = 15;
    }
}