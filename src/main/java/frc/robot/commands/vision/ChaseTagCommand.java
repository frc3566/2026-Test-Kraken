// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChaseTagCommand extends Command {
  /** Creates a new ChaseTagCommand. */
  // this is honestly useless but it cool for messing around and testing vision
  private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(0.1, 0.5);
  private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(0.1, 0.5);
  private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);

  private static final int tagToChase = 5;
  private static final Transform3d tagToGoal = 
      new Transform3d(1, 0, 0,
          new Rotation3d(0, 0, Math.PI)); // yaw may be -pi/2 if front hasnt changed
//   private final PhotonCamera photonCamera;
  private final CommandSwerveDrivetrain swerve;

  private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, yConstraints);
  private final ProfiledPIDController rotController = new ProfiledPIDController(1, 0, 0, rotConstraints);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  private PhotonTrackedTarget lastTarget;

  public ChaseTagCommand(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    rotController.setTolerance(Units.degreesToRadians(5));
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = swerve.getState().Pose;
    rotController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose2d = swerve.getState().Pose;
    var robotPose = 
        new Pose3d(
          robotPose2d.getX(),
          robotPose2d.getY(),
          0,
          new Rotation3d(0,0, robotPose2d.getRotation().getRadians())
        );
    Shuffleboard.getTab("Vision").add("Current Pose", String.format("(%.2f, %.2f) %.2f degrees", 
      robotPose2d.getX(),
      robotPose2d.getY(), 
      robotPose2d.getRotation().getDegrees()));
    
    var photonResOpt = Vision.Cameras.MAIN.getLatestResult();
    if(!photonResOpt.isEmpty()){
      var photonRes = Vision.Cameras.MAIN.getLatestResult().get();

      if (photonRes.hasTargets()) {
        var targetOpt = photonRes.getTargets().stream()
            .filter(t -> t.getFiducialId() == tagToChase)
            .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1) 
            .findFirst();
        if (targetOpt.isPresent()) {
          var target = targetOpt.get();
          Shuffleboard.getTab("Vision").addBoolean("isTargetFound", () -> targetOpt.isPresent());
          lastTarget = target;

          var cameraPose = robotPose.transformBy(Constants.Vision.robotToCamera);
          var camToTarget = target.getBestCameraToTarget();
          var targetPose = cameraPose.transformBy(camToTarget);
          var goalPose = targetPose.transformBy(tagToGoal).toPose2d();
          if (Math.abs(xController.getGoal().position - goalPose.getX()) > 0.05) {
            xController.setGoal(goalPose.getX());
          }
          if (Math.abs(yController.getGoal().position - goalPose.getY()) > 0.05) {
            yController.setGoal(goalPose.getY());
          }
          if (Math.abs(Rotation2d.fromRadians(rotController.getGoal().position).minus(goalPose.getRotation()).getRadians()) > 0.05) {
            rotController.setGoal(goalPose.getRotation().getRadians());
          }
        }
      }
      if (lastTarget == null) {
        // swerve.applyRequest(() ->
        //   drive.withVelocityX(0) // Drive forward with negative Y (forward)
        //       .withVelocityY(0) // Drive left with negative X (left)
        //       .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        // );
      } 
  } else {
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }
      

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      
      var rot = rotController.calculate(robotPose2d.getRotation().getRadians());
      if (rotController.atGoal()) {
        rot = 0;
      }
      
      // Final vars required for the applyRequest Command
      // Not sure if it is different from setControl
      final double xSupplier = xSpeed;
      final double ySupplier = ySpeed;
      final double rotSupplier = rot;

      // swerve.applyRequest(() ->
      //   drive.withVelocityX(xSupplier) // Drive forward with negative Y (forward)
      //        .withVelocityY(ySupplier) // Drive left with negative X (left)
      //        .withRotationalRate(rotSupplier) // Drive counterclockwise with negative X (left)
      // );
        Shuffleboard.getTab("Vision").add("Target Velocity", String.format("(%.2f, %.2f) %.2f degrees", 
          xSpeed,
          ySpeed, 
          rot));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      //   swerve.applyRequest(() ->
      //   drive.withVelocityX(0) // Drive forward with negative Y (forward)
      //        .withVelocityY(0) // Drive left with negative X (left)
      //        .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      // );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}