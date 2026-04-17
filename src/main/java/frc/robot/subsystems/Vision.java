package frc.robot.subsystems;

import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.Unit;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

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
      AprilTagFields.k2026RebuiltAndymark);
  /**
   * Ambiguity defined as a value between (0,1). Used in
   * {@link Vision#filterPose}.
   */
  private final double maximumAmbiguity = 0.15;
  /**
   * When false, {@link #updatePoseEstimation} is a no-op.
   * Use {@link #setPoseEstimationEnabled} to toggle.
   */
  private boolean poseEstimationEnabled = true;
  /**
   * Maximum rotation difference in degrees between vision pose estimates and current odometry pose, prevent "other side" issues
   */
  private final double maximumRotationDifferenceDeg = 180;
  /**
   * 
   * Photon Vision Simulation
   */
  public static VisionSystemSim visionSim;
  /**
   * Count of times that the odom thinks we're more than 10meters away from the
   * april tag.
   */
  private double longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Estimated robot pose from the vision system,
   */
  private Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference
   *                    {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose) {
    this.currentPose = currentPose;
    // this.field2d = field;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);
      visionSim.getDebugField();

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim, c.robotToCamTransform);
      }
      // openSimCameraViews();
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
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }

  /** 
   * @return the Pose2d from robot center to AprilTag target
   */

  // public static Transform2d getRobotRelativeTransformTo(PhotonTrackedTarget target) {
  //     Transform3d transform = target.getBestCameraToTarget();
  //     Translation2d end = transform.getTranslation().toTranslation2d()
  //         .plus(Constants.Vision.robotToCamera.getTranslation().toTranslation2d());

  //     double zAngleTheta = transform.getRotation().getZ();
  //     Rotation2d yaw = Rotation2d.fromRadians(Math.signum(zAngleTheta) * (Math.PI - Math.abs(zAngleTheta))).unaryMinus();

  //     return new Transform2d(end, yaw);
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

  public void updatePoseEstimation(CommandSwerveDrivetrain swerveDrive) {
    if (!poseEstimationEnabled) { return; }
    boolean anyPoseAvailable = false;
    boolean anyFilteredAvailable = false;

    for (Cameras camera : Cameras.values()) {
      String cameraKey = "Vision/" + camera.name();
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);

      SmartDashboard.putBoolean(cameraKey + "/Pose Estimation Available", poseEst.isPresent());
  anyPoseAvailable |= poseEst.isPresent();

      poseEst = filterPose(poseEst, camera.name());

      SmartDashboard.putBoolean(cameraKey + "/Filtered Pose Available", poseEst.isPresent());
  anyFilteredAvailable |= poseEst.isPresent();

      if(poseEst.isPresent()){
        var pose = poseEst.get().estimatedPose.toPose2d();
        swerveDrive.addVisionMeasurement(
              pose,
              poseEst.get().timestampSeconds,
              camera.curStdDevs);
        SmartDashboard.putString(cameraKey + "/Filtered Pose", pose.toString());
      }
    }

    // Keep aggregate booleans for existing dashboards/widgets.
    SmartDashboard.putBoolean("Vision/Pose Estimation Available", anyPoseAvailable);
    SmartDashboard.putBoolean("Vision/Filtered Pose Available", anyFilteredAvailable);
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the
   * given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  // public void updatePoseEstimation(CommandSwerveDrivetrain swerveDrive) {
  //   for (Cameras camera : Cameras.values()) {
  //     camera.updateUnreadResults();
  //     var poseEst = camera.getEstimatedGlobalPose();
  //     if(poseEst.isPresent()){
  //         var filteredPoseEst = filterPose(poseEst);
  //         if(filteredPoseEst.isPresent()){
  //           var pose = filteredPoseEst.get();
            
  //             SmartDashboard.putBoolean("Vision Measure Updating", true);
  //             // System.out.println("Updating pose estimation with camera measurement.");
  //         } else {
  //             // System.out.println("Camera measurement did not pass filter.");
  //         }
        
  //     } else {
  //       SmartDashboard.putBoolean("Vision Measure Updating", false);
  //     }
  //   }
  // }
      
   

  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose, String cameraName) {
    if (pose.isEmpty()) {
      String cameraKey = "Vision/" + cameraName;
      SmartDashboard.putString(cameraKey + "/Filter Reject Reason", "No pose estimate");
      SmartDashboard.putBoolean(cameraKey + "/Pose Too Far", false);
      return pose;
    }

  String cameraKey = "Vision/" + cameraName;

  // Keep only targets whose ambiguity is below the threshold.
  // Use a looser threshold (0.25) when multiple tags are in view; otherwise 0.15.
  double multiTagAmbiguity = 0.25;
  double singleTagAmbiguity = maximumAmbiguity; // 0.15 by default
  boolean multipleTagsInView = pose.get().targetsUsed.size() > 1;
  double activeThreshold = multipleTagsInView ? multiTagAmbiguity : singleTagAmbiguity;

  // Targets with ambiguity < 0 are ignored by PhotonVision (multi-tag result);
  // we pass those through unconditionally since they have no ambiguity value.
  var goodTargets = pose.get().targetsUsed.stream()
    .filter(t -> t.getPoseAmbiguity() < 0 || t.getPoseAmbiguity() < activeThreshold)
    .toList();

    SmartDashboard.putNumber(cameraKey + "/Good Targets Count", goodTargets.size());
    SmartDashboard.putNumber(cameraKey + "/Total Targets Count", pose.get().targetsUsed.size());
    SmartDashboard.putString(
      cameraKey + "/Passed Target IDs",
      goodTargets.stream()
        .map(target -> Integer.toString(target.getFiducialId()))
        .toList()
        .toString());

    // Log target counts and per-target ambiguity to the SignalLogger
    SignalLogger.writeDouble("Vision/" + cameraName + "/Targets/Total", pose.get().targetsUsed.size()-1, "count");
    SignalLogger.writeDouble("Vision/" + cameraName + "/Targets/Passed", goodTargets.size(), "count");
    // int targetIndex = 0;
    // for (var target : pose.get().targetsUsed) {
    //   SignalLogger.writeDouble(
    //     "Vision/" + cameraName + "/Targets/Ambiguity/Fid" + target.getFiducialId(),
    //     target.getPoseAmbiguity(),
    //     "unitless");
    //   SignalLogger.writeDouble(
    //     "Vision/" + cameraName + "/Targets/Ambiguity/Index" + targetIndex,
    //     target.getPoseAmbiguity(),
    //     "unitless");
    //   targetIndex++;
    // }

    if (goodTargets.isEmpty()) {
    SmartDashboard.putString(cameraKey + "/Filter Reject Reason",
      "All targets above ambiguity threshold (" + activeThreshold + ")");
      return Optional.empty();
    }

    // Rebuild the pose with only the passing targets so that downstream
    // std-dev calculations reflect only trustworthy observations.
    var filtered = new EstimatedRobotPose(
        pose.get().estimatedPose,
        pose.get().timestampSeconds,
        goodTargets,
        pose.get().strategy);

    // Reject if the vision heading is more than 90 degrees off from current odometry heading.
    double rotationDiffDeg = Math.abs(
        filtered.estimatedPose.toPose2d().getRotation()
            .minus(currentPose.get().getRotation())
            .getDegrees());
    SmartDashboard.putNumber(cameraKey + "/Rotation Diff to Odometry (deg)", rotationDiffDeg);
    if (rotationDiffDeg > maximumRotationDifferenceDeg) {
      SmartDashboard.putString(cameraKey + "/Filter Reject Reason", "Rotation Too Far: " + rotationDiffDeg + " deg");
      return Optional.empty();
    }

    // Reject if the vision estimate is too far from current odometry pose.
    double poseDiffTag = PhotonUtils.getDistanceToPose(currentPose.get(), filtered.estimatedPose.toPose2d());
    SmartDashboard.putNumber(cameraKey + "/Distance to Odometry Pose", poseDiffTag);
    if (poseDiffTag > 3.5) {
      longDistangePoseEstimationCount++;
      SmartDashboard.putNumber(cameraKey + "/Long Distance Pose Count", longDistangePoseEstimationCount);
      // Allow through only after many consecutive far readings (robot may be genuinely lost)
      if (longDistangePoseEstimationCount < 30) {
        SmartDashboard.putBoolean(cameraKey + "/Pose Too Far", true);
        SmartDashboard.putString(cameraKey + "/Filter Reject Reason", "Pose Too Far: " + poseDiffTag);
        return Optional.empty();
      }
    } else {
      longDistangePoseEstimationCount = 0;
      SmartDashboard.putNumber(cameraKey + "/Long Distance Pose Count", longDistangePoseEstimationCount);
      SmartDashboard.putBoolean(cameraKey + "/Pose Too Far", false);
    }

    SmartDashboard.putString(cameraKey + "/Filter Reject Reason", "Passed");
    return Optional.of(filtered);
  }

  /**ß
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   * <li>No Pose Estimates could be generated</li>
   * <li>The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and
   *         targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose(currentPose.get().getRotation());
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est -> debugField
              .getObject("VisionEstimation")
              .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;

  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running
   * photon vision on localhost.
   */
  private void openSimCameraViews() {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
      try
      {
      Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      } catch ( IOException | URISyntaxException e)
      {
      e.printStackTrace();
      }
    }
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    // field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to select each camera
   */
  public enum Cameras {
    /**
     * Ray (LL4), front
     * TODO: Set New Position
     */
    FRONT("RAY",
        new Rotation3d(0, -0.412, 0),
        new Translation3d(
          Units.inchesToMeters(13.957), // Front from center
          Units.inchesToMeters(0), //  Left from center
          Units.inchesToMeters(7.282) // Height from center
        ),
        VecBuilder.fill(0.75, 0.75, Units.degreesToRadians(360)), VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(360))),
    
    /**
     * Dingyi (LL3), Right
     * StdDev is higher b/c less accurate
     */
    RIGHT("DINGYI",
        new Rotation3d(0, -0.412,-1.571),
        new Translation3d(
          Units.inchesToMeters(5.5),
          Units.inchesToMeters(-12.289),
          Units.inchesToMeters(8.586)
        ),
        VecBuilder.fill(1, 1, Units.degreesToRadians(360)), VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(360))),

    /**
     * LEO (LL4), Left
     */
    LEFT("LEO",
        new Rotation3d(0, -0.412, 1.571),
        new Translation3d(
          Units.inchesToMeters(5.5),
          Units.inchesToMeters(12.289),
          Units.inchesToMeters(8.586)
        ),
        VecBuilder.fill(0.75, 0.75, Units.degreesToRadians(360)), VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(360)));

    
    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;
    /**
     * Camera instance for comms.
     */
    public final PhotonCamera camera;
    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     * X, Y, and Rotation respectively. Higher values means less trust.
     */
    private final Matrix<N3, N1> singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     * Check {@link poseEstimator} for more details on the meaning of values.
     */ 
    private final Matrix<N3, N1> multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of
     * the robot
     */
    private final Transform3d robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public Matrix<N3, N1> curStdDevs;
    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /**
     * Simulated camera instance which only exists during simulations.
     */
    public PhotonCameraSim cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary
     * queries.
     */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake
     * values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV
     *                              UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of
     *                              the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated
     *                              poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated
     *                              poses from the camera.
     */
    Cameras(
      String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
      Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix
    ) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(99.41));
        cameraProperties.setCalibError(1, 0.08);
        cameraProperties.setFPS(50);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim, Transform3d robotToCamera) {
      if (Robot.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamera); // uses real camera transform
      }
    }

    public PhotonCamera getCamera() {
      return camera;
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within
     * the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target.
     *         This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double ambiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < ambiguity && currentAmbiguity > 0) {
          bestResult = result;
          ambiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is
     *         there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      var temp = camera.getLatestResult();
      return temp.hasTargets() ? Optional.of(temp) : Optional.empty();
      // return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation,
     * standard deviations, and flushes the
     * cache of results.
     *
     * @param currentHeading Current field-relative robot heading from odometry.
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Rotation2d currentHeading) {
      updateUnreadResults();
      if (!resultsList.isEmpty()) {
        updateEstimatedGlobalPose(currentHeading);
      }
      return estimatedRobotPose;
    }

    /**
     * Update the latest results
     */
    public void updateUnreadResults() {
      resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
      // System.out.println("Latests Camera readings? " + resultsList.get(resultsList.size() - 1));
      
      /* 
        the following function is not getting ran because mostRecentTimestamp > currentTimestamp by a lot
        most likely mostRecentTimeStamp is wrong
      */

      // double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      // double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      // double debounceTime = Milliseconds.of(15).in(Seconds);

      // System.out.println("Result timestamps: " + resultsList.stream().map(e -> e.getTimestampSeconds()).toList());

      // System.out.println("Most recent: " + mostRecentTimestamp + " Last read: " + lastReadTimestamp + " Current: " + currentTimestamp);


      // if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
      //     (currentTimestamp - lastReadTimestamp) >= debounceTime) {
      //   System.out.println("Camera readings: " + camera.getAllUnreadResults());
      //   resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
      //   lastReadTimestamp = currentTimestamp;
      //   resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
      //     return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
      //   });
      //   if (!resultsList.isEmpty()) {
      //     updateEstimatedGlobalPose();
      //   }

      // } else{
      //     System.out.println("NO VISION UPDATE");
      //     System.out.println("Current - Most Recent:" + (currentTimestamp - mostRecentTimestamp));
      //     System.out.println("Current - Last Read:" + (currentTimestamp - lastReadTimestamp));
      //   }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should only be called once
     * per loop.
     *
     * <p>
     * Also includes updates for the standard deviations, which can (optionally) be
     * retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @param currentHeading Current field-relative robot heading from odometry.
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets used for
     *         estimation.
     */
    private void updateEstimatedGlobalPose(Rotation2d currentHeading) {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      SmartDashboard.putNumber("Vision/Heading For Pose Estimation (deg)", currentHeading.getDegrees());
      for (var change : resultsList) {
        poseEstimator.addHeadingData(change.getTimestampSeconds(), currentHeading);
        visionEst = poseEstimator.update(change); // Uses MULTI_TAG_PNP_ON_COPROCESSOR, falls back to PNP_DISTANCE_TRIG_SOLVE
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      // System.out.println("Localization estimation: " + visionEst.map(e -> e.estimatedPose.toPose2d()));
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) {
            continue;
          }
          numTags++;
          avgDist += tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          // Do not trust vision if only one tag and far away
          if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
          SmartDashboard.putString("Vision/StdDevs", curStdDevs.toString());
        }
      }
    }
  }
}
