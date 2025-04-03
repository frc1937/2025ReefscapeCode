package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Standard Deviations represent how ambiguous an estimated pose is using calibrated gains.
 * The higher the value is, the more ambiguous the pose is and the less trustworthy the result.
 * Standard Deviations are used to reduce noise in a pose estimate result by accounting for how much each result is wrong by.
 */
public record StandardDeviations(double translationStandardDeviation, double thetaStandardDeviation) {
    /**
     * Constructs an object that stores how ambiguous the estimated pose of the robot is.
     * The greater the number, the less trustworthy the pose is.
     *
     * @param translationStandardDeviation the ambiguity of the translation aspect of the pose estimation
     * @param thetaStandardDeviation       the ambiguity of the rotation aspect of the pose estimation
     */
    public StandardDeviations {
    }

    public Matrix<N3, N1> toMatrix() {
        return VecBuilder.fill(
                translationStandardDeviation,
                translationStandardDeviation,
                thetaStandardDeviation
        );
    }
}
