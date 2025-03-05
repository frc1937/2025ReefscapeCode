package frc.robot.subsystems.climb;

import frc.lib.generic.hardware.motor.*;
import frc.robot.utilities.PortsConstants;

public class ClimbConstants {
    public enum ClimbState {
        INITIAL_STATE(0),
        READY_CLIMB(2);

        public final double positionRotations;

        ClimbState(double positionRotations) {
            this.positionRotations = positionRotations;
        }
    }

    public static Motor CLIMB_MOTOR = MotorFactory.createSpark("CLIMBER", PortsConstants.ClimbPorts.CLIMB_MOTOR_PORT, MotorProperties.SparkType.MAX);

    static {
        configureClimbMotor();
    }

    private static void configureClimbMotor() {
        final MotorConfiguration climbConfiguration = new MotorConfiguration();

        climbConfiguration.inverted = true;
        climbConfiguration.gearRatio = 87.5;
        climbConfiguration.supplyCurrentLimit = 40;
        climbConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        climbConfiguration.slot = new MotorProperties.Slot(1,0,0,0,0,0);

        CLIMB_MOTOR.configure(climbConfiguration);

        CLIMB_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        CLIMB_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        CLIMB_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        CLIMB_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}
