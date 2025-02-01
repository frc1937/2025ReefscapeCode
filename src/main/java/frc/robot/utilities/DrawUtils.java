package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public class DrawUtils {
    /**
     * Draws a circle made out of Pose2d.
     *
     * @param radius The radius of the circle.
     * @param centerPose The center pose of the circle.
     * @param numPoints The number of points to approximate the circle.
     */
    public static void drawCircle(double radius, Pose2d centerPose, int numPoints) {
        final Pose2d[] circlePoses = new Pose2d[numPoints];

        for (int i = 0; i < numPoints; i++) {
            final double theta = 2 * Math.PI * i / (numPoints - 1);

            final Translation2d radial = new Translation2d(
                    radius * Math.cos(theta),
                    radius * Math.sin(theta)
            ).rotateBy(centerPose.getRotation());


            circlePoses[i] = new Pose2d(
                    centerPose.getTranslation().plus(radial),
                    new Rotation2d(radial.getX(), radial.getY()).plus(Rotation2d.fromDegrees(90))
            );
        }

        Logger.recordOutput("DrawingUtils/", circlePoses);
    }

}