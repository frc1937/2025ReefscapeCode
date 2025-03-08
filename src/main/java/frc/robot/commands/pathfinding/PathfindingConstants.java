package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utilities.FieldConstants;

public class PathfindingConstants {
    public enum Branch {
        LEFT_BRANCH {
            @Override
            public Pose2d getBranchPose() {
                return PathfindingCommands.decideReefFace().getLeftBranch();
            }

            @Override
            public Pose2d getBranchPose(FieldConstants.ReefFace face) {
                return face.getLeftBranch();
            }
        },

        RIGHT_BRANCH {
            @Override
            public Pose2d getBranchPose() {
                return PathfindingCommands.decideReefFace().getRightBranch();
            }

            @Override
            public Pose2d getBranchPose(FieldConstants.ReefFace face) {
                return face.getRightBranch();
            }
        };

        /**
         * Get the nearest branch pose
         *
         * @return The branch pose
         */
        public abstract Pose2d getBranchPose();

        /**
         * Get the branch pose for a specific reef face, alliance compensated.
         *
         * @param face The reef face
         * @return The branch pose
         */
        public abstract Pose2d getBranchPose(FieldConstants.ReefFace face);
    }
}
