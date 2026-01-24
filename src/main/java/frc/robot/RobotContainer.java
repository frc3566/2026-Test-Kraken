// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.TestCommands;
import frc.robot.commands.vision.GetVisionData;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        if(DriverStation.isTeleop()){
            drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // joystick.setRumble(kBothRumble,0.5);
        }
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.y().onTrue(new InstantCommand(() -> {        
            Vision.Cameras.MAIN.updateUnreadResults();
            var results = Vision.Cameras.MAIN.getCamera().getAllUnreadResults();    
            if(results.isEmpty()){
                System.out.println("No targets detected.");
            }
            else{
                var result = results.get(0);
                // At least one AprilTag was seen by the camera
                if (result.hasTargets()) {
                    System.out.println("Targets detected.");
                    System.out.println(result.getBestTarget().fiducialId);
                    // System.out.println(result.metadata.captureTimestampMicros);
                    }
                else{
                    System.out.println("What happend?");
                    // System.out.println(result.metadata.captureTimestampMicros);
                }
            }            
        }));

        joystick.x().onTrue(new GetVisionData());

        

        // reset the field-centric heading on left bumper press
        // joystick.y().whileTrue(new InstantCommand(() -> {
        //     System.out.println("pressed x");
        //     Vision.Cameras.MAIN.updateUnreadResults();
        //     var results = Vision.Cameras.MAIN.getCamera().getAllUnreadResults();
        //     var result = results.get(results.size() - 1);
        //         // At least one AprilTag was seen by the camera
        //     if (result.hasTargets()) {
        //         var target = result.getBestTarget();
        //         var transform = Vision.getRobotRelativeTransformTo(target);
        //         // System.out.println("Best target ID: " + target.getFiducialId());
        //          Shuffleboard.getTab("Vision").addNumber("Best target ID:", () -> target.getFiducialId()); 
        //         // System.out.println("Robot-relative transform to target: " + transform);
        //     } else {
        //         // System.out.println("No targets seen");
        // }
        //     // Vision.Cameras.MAIN.getBestResult()
        //     //   .map(e -> (e.hasTargets() ? e.getBestTarget() : null))
        //     //   .map(Vision::getRobotRelativeTransformTo)
        //     //   .ifPresent(System.out::println);
        // }).repeatedly());
        
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
