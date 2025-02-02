package frc.lib.generic.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public class DrawUtils {
    private static final double TWO_PI = 2 * Math.PI;
    private static final String LOG_DIRECTORY = "DrawingUtils/";

    /**
     * Draws a circle made out of Pose2d.
     *
     * @param radius The radius of the circle.
     * @param centerPose The center pose of the circle.
     * @param numPoints The number of points to approximate the circle.
     * @param name The name as it's seen in AdvantageScope
     */
    public static void drawCircle(double radius, Pose2d centerPose, int numPoints, String name) {
        final Pose2d[] circlePoses = new Pose2d[numPoints];

        for (int i = 0; i < numPoints; i++) {
            final double theta = TWO_PI * i / (numPoints - 1);

            final Translation2d radial = new Translation2d(
                    radius * Math.cos(theta),
                    radius * Math.sin(theta)
            ).rotateBy(centerPose.getRotation());

            circlePoses[i] = new Pose2d(
                    centerPose.getTranslation().plus(radial),
                    new Rotation2d(radial.getX(), radial.getY()).plus(Rotation2d.fromDegrees(90))
            );
        }

        Logger.recordOutput(LOG_DIRECTORY + name, circlePoses);
    }

    /**
     * Draws a line between two points.
     *
     * @param startPoint The starting point of the line.
     * @param endPoint The ending point of the line.
     * @param name The name as it's seen in AdvantageScope
     */
    public static void drawLine(Translation2d startPoint, Translation2d endPoint, String name) {
        Pose2d[] linePoses = new Pose2d[2];

        linePoses[0] = new Pose2d(startPoint, Rotation2d.kZero);
        linePoses[1] = new Pose2d(endPoint, Rotation2d.kZero);

        Logger.recordOutput(LOG_DIRECTORY + name, linePoses);
    }
}