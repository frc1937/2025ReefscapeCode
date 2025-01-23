package frc.lib.generic.simulation.extensions;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ExtendedDCMotorSim extends DCMotorSim {
    public ExtendedDCMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox) {
        super(plant, gearbox);
    }

    /**
     * Returns the acceleration of the motor.
     *
     * @return The acceleration of the motor in radians per second squared.
     */
    public double getAccelerationRadiansPerSecondSquared() {
        // Get the current state vector (position and velocity)
        Matrix<N2, N1> x = m_x;

        // Get the input voltage
        Matrix<N1, N1> u = m_u;

        // Compute the state derivative (xdot = A * x + B * u)
        Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(u));

        // The acceleration is the second element of the state derivative vector
        return xdot.get(1, 0);
    }
}
