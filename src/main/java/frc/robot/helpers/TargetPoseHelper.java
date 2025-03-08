package frc.robot.helpers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap;
import java.util.Optional;

/**
 * The TargetPoseHelper class is a helper class that provides methods to calculate the target pose
 * for the robot to drive to.
 */
public class TargetPoseHelper {

  private static AprilTagFieldLayout m_fieldLayout;
  private static Alliance m_allianceColor;
  private static Pose2d m_redReefCenter = new Pose2d(0.0, 0.0, new Rotation2d());
  private static Pose2d m_blueReefCenter = new Pose2d(0.0, 0.0, new Rotation2d());

  public TargetPoseHelper() {
    m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // the difference between tag 18 and tag 21 is the center of the blue reef
    Pose2d blue1;
    Pose2d blue2;
    Pose2d red1;
    Pose2d red2;

    Optional<Pose3d> temp = m_fieldLayout.getTagPose(18);
    if (temp.isPresent()) {
      blue1 = temp.get().toPose2d();
    } else {
      blue1 = new Pose2d();
    }
    temp = m_fieldLayout.getTagPose(21);
    if (temp.isPresent()) {
      blue2 = temp.get().toPose2d();
    } else {
      blue2 = new Pose2d();
    }

    // halway between is the center of the blue reef
    double diff = blue2.getX() - blue1.getX();
    m_blueReefCenter = new Pose2d(blue1.getX() + diff / 2, blue1.getY(), new Rotation2d());

    // the different between tag 7 and tag 10 is the center of the red reef
    temp = m_fieldLayout.getTagPose(7);
    if (temp.isPresent()) {
      red1 = temp.get().toPose2d();
    } else {
      red1 = new Pose2d();
    }
    temp = m_fieldLayout.getTagPose(10);
    if (temp.isPresent()) {
      red2 = temp.get().toPose2d();
    } else {
      red2 = new Pose2d();
    }

    // halway between is the center of the red reef
    diff = red1.getX() - red2.getX();
    m_redReefCenter = new Pose2d(red2.getX() + diff / 2, red2.getY(), new Rotation2d());
  }

  public void setAllianceColor(Alliance color) {
    if (m_allianceColor != color) {
      m_allianceColor = color;
    }
  }

  /**
   * Calculate the target pose for the robot to drive to.
   *
   * @param tagPose The pose of the target
   * @param offsetX The x offset from the target
   * @param offsetY The y offset from the target
   * @return The target pose for the robot to drive to
   */
  public static Pose2d calculateTargetPose(
      RobotMap.FieldConstants.TAG_IDS tagID,
      RobotMap.FieldConstants.REEF_OFFSETS fieldElementOffset) {

    // Get the Pose of the tag we're focused on
    Optional<Pose3d> curTagPose = m_fieldLayout.getTagPose(tagID.getValue());
    Pose3d newPose = new Pose3d();

    if (curTagPose.isPresent()) {
      // We need to move to the right or the left based on the fieldElementOffset
      // requested
      if (fieldElementOffset == RobotMap.FieldConstants.REEF_OFFSETS.RIGHT_BRANCH) {
        newPose =
            curTagPose
                .get()
                .transformBy(RobotMap.FieldConstants.FIELD_TRANSFORMS.RIGHT_BRANCH)
                .transformBy(RobotMap.FieldConstants.FIELD_TRANSFORMS.ROBOT_TRANSFORM);
      } else {
        newPose =
            curTagPose
                .get()
                .transformBy(RobotMap.FieldConstants.FIELD_TRANSFORMS.LEFT_BRANCH)
                .transformBy(RobotMap.FieldConstants.FIELD_TRANSFORMS.ROBOT_TRANSFORM);
      }
    }
    System.out.println("Calculate Target Pose: [" + newPose.getX() + "][" + newPose.getY() + "]");
    return newPose.toPose2d();
  }

  private static double CalcDistanceToAprilTag(
      RobotMap.FieldConstants.TAG_IDS tag, Pose2d currentPose) {

    System.out.println("CalcDistanceToAprilTag: [" + tag.getValue() + "]");
    // Get the Pose of the tag we're focused on
    Optional<Pose3d> curTagPose = m_fieldLayout.getTagPose(tag.getValue());

    if (curTagPose.isPresent()) {
      Pose2d aprilTagPose = curTagPose.get().toPose2d();
      return currentPose.getTranslation().getDistance(aprilTagPose.getTranslation());
    }
    return 0.0;
  }

  public static RobotMap.FieldConstants.TAG_IDS GetNearestReefTagID(Pose2d curPose) {
    double distance = 0.0;
    double chassisX = curPose.getX();
    if (m_allianceColor == Alliance.Red) {

      // Determine which side of the reef the robot is on by comparing X position to the x position
      // of the center of the reef.
      // This will tell us which side of the reef can potentially have the closes face.  Then
      // calculate the distance to the face
      // that is parallel to the alliance wall.  Compare that distance to one of the other sides of
      // the reef that is on that half of
      // the reef.  If it is lower, then that face is the closest, if not compare to the other face
      // on that half of the reef.  It this face
      // or the parallel face that will be the closest.
      //
      // So, in the picture below, first determine if it is either abc or def that are the potential
      // closest faces.
      // Then, first calculate the distance of either b or e.  Now if you compare a or d and it is
      // closer that b or e, then it is closer.
      // Otherwise, we need to compare c or f to see if they are closer.
      //
      //                                        *
      //                                   a  *   * d
      //                                     #     #
      //                                 b   |  +  | e
      //                                     #     #
      //                                  c   *   * f
      //                                        *
      //
      System.out.println(
          "Trying to find nearest Reef face Pose: ["
              + curPose.getX()
              + "]["
              + curPose.getY()
              + "]");
      if (chassisX >= m_redReefCenter.getX()) {
        distance = CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.RED_REEF_AB_TAG, curPose);
        if (CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.RED_REEF_CD_TAG, curPose)
            < distance) {
          return RobotMap.FieldConstants.TAG_IDS.RED_REEF_CD_TAG;
        } else if (CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.RED_REEF_KL_TAG, curPose)
            < distance) {
          return RobotMap.FieldConstants.TAG_IDS.RED_REEF_KL_TAG;
        }
        return RobotMap.FieldConstants.TAG_IDS.RED_REEF_AB_TAG;
      } else {
        distance = CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.RED_REEF_GH_TAG, curPose);
        if (CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.RED_REEF_EF_TAG, curPose)
            < distance) {
          return RobotMap.FieldConstants.TAG_IDS.RED_REEF_EF_TAG;
        } else if (CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.RED_REEF_IJ_TAG, curPose)
            < distance) {
          return RobotMap.FieldConstants.TAG_IDS.RED_REEF_IJ_TAG;
        }
        return RobotMap.FieldConstants.TAG_IDS.RED_REEF_GH_TAG;
      }
    } else {
      if (chassisX <= m_blueReefCenter.getX()) {
        distance =
            CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_AB_TAG, curPose);
        if (CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_CD_TAG, curPose)
            < distance) {
          return RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_CD_TAG;
        } else if (CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_KL_TAG, curPose)
            < distance) {
          return RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_KL_TAG;
        }
        return RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_AB_TAG;
      } else {
        distance =
            CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_GH_TAG, curPose);
        if (CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_EF_TAG, curPose)
            < distance) {
          return RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_EF_TAG;
        } else if (CalcDistanceToAprilTag(RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_IJ_TAG, curPose)
            < distance) {
          return RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_IJ_TAG;
        }
        return RobotMap.FieldConstants.TAG_IDS.BLUE_REEF_GH_TAG;
      }
    }
  }

  public Pose2d getTargetPoseForRightReef(Pose2d curPose) {

    RobotMap.FieldConstants.TAG_IDS curTag;

    curTag = TargetPoseHelper.GetNearestReefTagID(curPose);

    // Calculate the target pose
    return TargetPoseHelper.calculateTargetPose(
        curTag, RobotMap.FieldConstants.REEF_OFFSETS.RIGHT_BRANCH);
  }

  public Pose2d getTargetPoseForLeftReef(Pose2d curPose) {

    RobotMap.FieldConstants.TAG_IDS curTag;

    curTag = TargetPoseHelper.GetNearestReefTagID(curPose);

    // Calculate the target pose
    return TargetPoseHelper.calculateTargetPose(
        curTag, RobotMap.FieldConstants.REEF_OFFSETS.LEFT_BRANCH);
  }
}
