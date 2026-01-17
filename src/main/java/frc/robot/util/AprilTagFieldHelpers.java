package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * A static helper class for processing the current AprilTagFieldLayout.
*/
public final class AprilTagFieldHelpers {
    private static final AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeWelded
    );

    private static final ArrayList<Pose2d> m_reefAprilTagPositions = new ArrayList<>(12);
    private static final ArrayList<Pose2d> m_stationAprilTagPositions = new ArrayList<>(4);

    // A static initializer for populating the april tag position lists.
    static {
        for (AprilTag aprilTag : m_aprilTagFieldLayout.getTags()) {
            int tagID = aprilTag.ID;

            // Note: 6-11 and 17-22 are the red and blue reef tag ID ranges.
            if ((6 <= tagID && tagID <= 11) || (17 <= tagID && tagID <= 22)) {
                m_reefAprilTagPositions.add(aprilTag.pose.toPose2d());
            }

            // Note: 1-2 and 12-13 are the red and blue coral station tag ID ranges.
            else if ((tagID == 1 || tagID == 2) || (tagID == 12 || tagID == 13)) {
                m_stationAprilTagPositions.add(aprilTag.pose.toPose2d());
            }
        }
    }

    /**
     * Returns the position of the nearest AprilTag on either sides' reef as a Pose2d.
     * 
     * @param currentPosition The Pose2d from which the nearest Pose2d is returned.
     * @return The nearest Pose2d in the {@link #m_reefAprilTagPositions} to the {@code currentPosition}
    */
    public static final Pose2d getNearestReefPose(Pose2d currentPosition) {
        return currentPosition.nearest(m_reefAprilTagPositions);
    }

    /**
     * Returns the position of the nearest AprilTag on either sides' coral stations as a Pose2d.
     * 
     * @param currentPosition The Pose2d from which the nearest Pose2d is returned.
     * @return The nearest Pose2d in the {@link #m_stationAprilTagPositions} to the {@code currentPosition}
    */
    public static final Pose2d getNearestStationPose(Pose2d currentPosition) {
        return currentPosition.nearest(m_stationAprilTagPositions);
    }
}
