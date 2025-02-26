package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Standard Deviations represent how ambiguous an estimated pose is using calibrated gains.
 * The higher the value is, the more ambiguous the pose is and the less trustworthy the result.
 * Standard Deviations are used to reduce noise in a pose estimate result by accounting for how much each result is wrong by.
 */
public record StandardDeviations(double translationStandardDeviation, double thetaStandardDeviation) {
    /**
     * Combines this with another {@link StandardDeviations}.
     * This might be used when estimating a pose in relation to another estimated pose.
     * In a case like this, you would want to find the combined standard deviations of both estimated poses to find the final standard deviations.
     *
     * @param other the {@link StandardDeviations} to combine with
     * @return the combined {@link StandardDeviations}
     */
    public StandardDeviations combineWith(StandardDeviations other) {
        return new StandardDeviations(
                combineStandardDeviation(translationStandardDeviation, other.translationStandardDeviation),
                combineStandardDeviation(thetaStandardDeviation, other.thetaStandardDeviation)
        );
    }



    public Matrix<N3, N1> toMatrix() {
        return VecBuilder.fill(
                translationStandardDeviation,
                translationStandardDeviation,
                thetaStandardDeviation
        );
    }

    public Transform2d scaleTransformFromStandardDeviations(Transform2d transform) {
        return new Transform2d(
                transform.getX() * translationStandardDeviation,
                transform.getY() * translationStandardDeviation,
                transform.getRotation().times(thetaStandardDeviation)
        );
    }

    /**
     * Combines two standard deviation values.
     *
     * @param firstStandardDeviation  the original standard deviation value
     * @param secondStandardDeviation the standard deviation value to combine with
     * @return the combined standard deviation
     */
    private double combineStandardDeviation(double firstStandardDeviation, double secondStandardDeviation) {
        if (firstStandardDeviation == 0.0)
            return 0.0;

        final double squaredSecondStandardDeviation = secondStandardDeviation * secondStandardDeviation;
        final double combinedSquareRoot = Math.sqrt(firstStandardDeviation * squaredSecondStandardDeviation);
        final double denominator = firstStandardDeviation + combinedSquareRoot;
        return firstStandardDeviation / denominator;
    }
}
