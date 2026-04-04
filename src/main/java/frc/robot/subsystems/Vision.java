package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken
 * from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision extends SubsystemBase {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2026RebuiltWelded);
  private static final List<String> LIMELIGHT_NAMES = List.of("limelight-2026-1");

  private boolean poseEstimationEnabled = true;
  private boolean updatePoseEstimation = true;

  public static VisionSystemSim visionSim;

  private final Supplier<Pose2d> currentPose;
  private final List<Limelight> limelights = new ArrayList<>();
  private final List<LimelightPoseEstimator> limelightPoseEstimators = new ArrayList<>();

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference
   *                    {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose) {
    this.currentPose = currentPose;

    for (String limelightName : LIMELIGHT_NAMES) {
      Limelight limelight = new Limelight(limelightName);
      limelights.add(limelight);
      limelightPoseEstimators.add(limelight.createPoseEstimator(EstimationMode.MEGATAG2));
    }

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */

  // public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
  //   Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
  //   if (aprilTagPose3d.isPresent()) {
  //     return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
  //   } else {
  //     throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
  //   }
  // }

  /**
   * Update pose estimation inside of {@link SwerveDrive} with all of the
   * given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  /** Enables or disables vision pose estimation updates. */
  public void setPoseEstimationEnabled(boolean enabled) {
    poseEstimationEnabled = enabled;
    SmartDashboard.putBoolean("Vision/Pose Estimation Enabled", enabled);
  }

  public boolean isPoseEstimationEnabled() {
    return poseEstimationEnabled;
  }

  public void seedInternalImu(CommandSwerveDrivetrain swerveDrive) {
    var pigeon = swerveDrive.getPigeon2();
    for (Limelight limelight : limelights) {
      limelight.getSettings().withImuMode(ImuMode.SyncInternalImu)
          .withRobotOrientation(
              new Orientation3d(
                  swerveDrive.getRotation3d(),
                  pigeon.getAngularVelocityZDevice().getValue(),
                  pigeon.getAngularVelocityYDevice().getValue(),
                  pigeon.getAngularVelocityXDevice().getValue()))
          .save();
    }
  }

  public void updatePoseEstimation(CommandSwerveDrivetrain swerveDrive) {
    if (!poseEstimationEnabled) { return; }
    var pigeon = swerveDrive.getPigeon2();

    for (int i = 0; i < limelights.size(); i++) {
      Limelight limelight = limelights.get(i);
      LimelightPoseEstimator estimator = limelightPoseEstimators.get(i);

      // Required for MegaTag2 localization.
      limelight.getSettings()
      .withRobotOrientation(
          new Orientation3d(
              swerveDrive.getRotation3d(),
              pigeon.getAngularVelocityZDevice().getValue(),
              pigeon.getAngularVelocityYDevice().getValue(),
              pigeon.getAngularVelocityXDevice().getValue()))
      .withImuMode(ImuMode.InternalImuExternalAssist)
          .save();
          
      Optional<PoseEstimate> poseEstimate = estimator.getPoseEstimate();
      boolean poseValid = poseEstimate.isPresent() && poseEstimate.get().hasData;
      SmartDashboard.putBoolean("Vision/Pose Estimation Available", poseValid);
      if (!poseValid) {
        continue;
      }

      PoseEstimate mt2Estimate = poseEstimate.get();

      Matrix<N3, N1> stdDevs =VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(3600));

      if (mt2Estimate.avgTagDist > 4.0) {
        stdDevs = stdDevs.times(1.5);
      }
      if (mt2Estimate.rawFiducials[0].ambiguity > 0.5 ||
          mt2Estimate.tagCount == 0) {
        updatePoseEstimation = false;
      }

      Pose2d visionPose = mt2Estimate.pose.toPose2d();

      if (updatePoseEstimation) {
        swerveDrive.addVisionMeasurement(visionPose, mt2Estimate.timestampSeconds, stdDevs);
      }

      SmartDashboard.putBoolean("Vision/Pose Estimation Update Applied", updatePoseEstimation);
      SmartDashboard.putString("Vision/Filtered Pose", visionPose.toString());
      SmartDashboard.putNumber("Vision/Limelight Tag Count", mt2Estimate.tagCount);
      SmartDashboard.putNumber("Vision/Limelight Avg Tag Dist", mt2Estimate.avgTagDist);
    }
  }

  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag
        .map(pose3d -> currentPose.get().getTranslation().getDistance(pose3d.toPose2d().getTranslation()))
        .orElse(-1.0);
  }

  public VisionSystemSim getVisionSim() {
    return visionSim;
  }
}
