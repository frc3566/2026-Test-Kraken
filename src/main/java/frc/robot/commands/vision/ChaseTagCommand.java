// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/**
 * Controls the robot to drive to a specific apriltag.java. Still WIP
 */
public class ChaseTagCommand extends Command {
  /** Creates a new ChaseTagCommand. */
  // this is honestly useless but it cool for messing around and testing vision
  private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 3);
  private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 3);
  private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(3/4 * Math.PI, 1/2 * Math.PI);

  private final int tagToChase;

  // 1 meter in front of the tag, facing the tag
  private static final Transform3d tagToGoal = 
      new Transform3d(1, 0, 0,
          new Rotation3d(0, 0, Math.PI)); // yaw may be -pi/2 if front hasnt changed
//   private final PhotonCamera photonCamera;
  private final CommandSwerveDrivetrain swerve;

  private final ProfiledPIDController xController = new ProfiledPIDController(0.5, 0, 0, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(0.5, 0, 0, yConstraints);
  private final ProfiledPIDController rotController = new ProfiledPIDController(2, 0, 0, rotConstraints);
  private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();
  private double xSpeed, ySpeed, rotSpeed;
  private PhotonTrackedTarget lastTarget;

    public ChaseTagCommand(CommandSwerveDrivetrain swerve, int tagToChase) {
    this.swerve = swerve;
    this.tagToChase = tagToChase;
    this.addRequirements(swerve);

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    rotController.setTolerance(Units.degreesToRadians(10));
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ChaseTag Command initiated.");
    lastTarget = null;
    // swerve.resetPose(new Pose2d());
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
    
      SmartDashboard.putString("Vision/Current Pose", String.format("(%.2f, %.2f) %.2f degrees", 
      robotPose2d.getX(),
      robotPose2d.getY(), 
      robotPose2d.getRotation().getDegrees()));

    
    var photonResOpt = Vision.Cameras.MAIN.getLatestResult();
    if(!photonResOpt.isEmpty()){
      var photonRes = photonResOpt.get();

      if (photonRes.hasTargets()) {
        var targetOpt = photonRes.getTargets().stream()
            .filter(t -> t.getFiducialId() == tagToChase)
            .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1) 
            .findFirst();
        if (targetOpt.isPresent()) {
          var target = targetOpt.get();
          SmartDashboard.putBoolean("Vision/isTargetFound", true);
          lastTarget = target;

          var cameraPose = robotPose.transformBy(Constants.Vision.robotToCamera);
          var camToTarget = target.getBestCameraToTarget();
          var targetPose = cameraPose.transformBy(camToTarget);
          var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

          SmartDashboard.putBoolean("Vision/xAtGoal", xController.atGoal());
          SmartDashboard.putBoolean("Vision/yAtGoal", yController.atGoal());
          SmartDashboard.putBoolean("Vision/rotAtGoal", rotController.atGoal());

          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          rotController.setGoal(goalPose.getRotation().getRadians());
        }
      }


        xSpeed = MathUtil.clamp(xController.calculate(robotPose.getX()), -3, 3);
        if (xController.atGoal()) {
          xSpeed = 0;
        }

        ySpeed = MathUtil.clamp(yController.calculate(robotPose.getY()), -3, 3);
        if (yController.atGoal()) {
          ySpeed = 0;
        }
        
        rotSpeed = MathUtil.clamp(rotController.calculate(robotPose2d.getRotation().getRadians()), -3, 3);
        if (rotController.atGoal()) {
          rotSpeed = 0;
        

        // swerve.setControl(
        //   drive.withVelocityX(MathUtil.clamp(xSpeed, -0.2, 0.2)) // Drive forward with negative Y (forward)
        //        .withVelocityY(MathUtil.clamp(ySpeed, -0.2, 0.2)) // Drive left with negative X (left)
        //        .withRotationalRate(MathUtil.clamp(rot, -Math.PI/12, Math.PI/12)) // Drive counterclockwise with negative X (left)
        // );
        
        SmartDashboard.putString("Vision/Target Velocity", String.format("(%.2f, %.2f) %.2f radians", 
        xSpeed,
        ySpeed, 
        rotSpeed));
      }
    } 
    else{
      System.out.println("Target not Found!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("ChaseTag Command Ended. Interrupted: " + interrupted);
        // swerve.applyRequest(() ->
        //   drive.withSpeeds(new ChassisSpeeds(0, 0, 0))
        // );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lastTarget == null || (xController.atGoal() && yController.atGoal() && rotController.atGoal());
  }
}