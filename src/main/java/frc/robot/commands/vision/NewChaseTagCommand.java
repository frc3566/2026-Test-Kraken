package frc.robot.commands.vision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class NewChaseTagCommand extends Command {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);

  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(4, 4);
  
  private static final int TAG_TO_CHASE = 4;
  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180.0));

  private final PhotonCamera photonCamera;
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(4, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);
  private static final Transform3d robotToCamera = Constants.Vision.robotToCamera;
  private static final Transform2d robotToCamera2d = new Transform2d(
    robotToCamera.getTranslation().toTranslation2d(),
    robotToCamera.getRotation().toRotation2d()
  );

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  public NewChaseTagCommand(
        CommandSwerveDrivetrain drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
    this.photonCamera = Vision.Cameras.MAIN.getCamera();
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    goalPose = null;
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    System.out.println("Executing chase tag command");
    var robotPose = poseProvider.get();
    var photonResList = photonCamera.getAllUnreadResults();
    var photonRes = !photonResList.isEmpty() ? photonResList.get(photonResList.size() - 1) : null;
    
    if (photonRes!=null && photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        if (!target.equals(lastTarget)) {
          // This is new target data, so recalculate the goal
          lastTarget = target;

          // Get the transformation from the camera to the tag (in 2d)
          var camToTarget = target.getBestCameraToTarget();
          var transform = new Transform2d(
            camToTarget.getTranslation().toTranslation2d(),

            // Why minus?
            camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
            
            // Transform the robot's pose to find the tag's pose
            var cameraPose = robotPose.transformBy(robotToCamera2d.inverse());
            Pose2d targetPose = cameraPose.transformBy(transform);
            
            // Transform the tag's pose to set our goal
            goalPose = targetPose.transformBy(TAG_TO_GOAL);
        }

        if (goalPose != null) {
          // Drive
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
    }
    
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }
    
    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
    if (omegaController.atGoal()) {
      omegaSpeed = 0;
    }

    // drivetrainSubsystem.driveRobotRelative(
    //   ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation())
    // );

        SmartDashboard.putBoolean("Vision/isTargetFound", goalPose != null);
        SmartDashboard.putString("Vision/xDiff", String.format("%.2f meters", (goalPose != null) ? Math.abs(xController.getGoal().position - robotPose.getX()) : 0));
        SmartDashboard.putString("Vision/yDiff", String.format("%.2f meters", (goalPose != null) ? Math.abs(yController.getGoal().position - robotPose.getY()) : 0));
        SmartDashboard.putString("Vision/rotDiff", String.format("%.2f rads", (goalPose != null) ? Math.abs(omegaController.getGoal().position - robotPose.getRotation().getRadians()) : 0));
        SmartDashboard.putNumber("Vision/X Speed", xSpeed);
        SmartDashboard.putNumber("Vision/Y Speed", ySpeed);
        SmartDashboard.putNumber("Vision/Omega Speed", omegaSpeed);
    }




  @Override
  public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    };

    @Override
    public boolean isFinished() {
        return ((xController.atGoal() && yController.atGoal() && omegaController.atGoal() && goalPose != null) || lastTarget == null);
  }
}