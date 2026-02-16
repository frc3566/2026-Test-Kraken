// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.ArmToSetpoint;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private boolean enableDrive = true;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController firstDriver = new CommandXboxController(0);
        private final CommandXboxController secondDriver = new CommandXboxController(1);
        

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    // public final Climber climber = new Climber();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.y().onTrue(new GetVisionData());

        // joystick.x().onTrue(new ChaseTagCommand(drivetrain));


        /* SysID stuffs */
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        if(enableDrive){
                drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-firstDriver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-firstDriver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-firstDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
        }
            
        firstDriver.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        firstDriver.leftBumper().onTrue(new InstantCommand( () -> shooter.setLowerPower(0.5)));
        firstDriver.leftBumper().onFalse(new InstantCommand( () -> shooter.stopLower()));
        // firstDriver.rightTrigger().onTrue(new ChaseTagCommand(drivetrain)); //TODO: Auto align+move to scoring pos
        // firstDriver.rightBumper().onTrue(new HeadingToHub()); //TODO: Heading snap to hub


        // Intake arm bindings
        // firstDriver.rightBumper().onTrue(new InstantCommand(() -> intake.armUp(0.1)));
        // firstDriver.rightBumper().onFalse(new InstantCommand(() -> intake.armStop()));
        // firstDriver.rightTrigger().onTrue(new InstantCommand(() -> intake.armDown(0.1)));
        // firstDriver.rightTrigger().onFalse(new InstantCommand(() -> intake.armStop()));

        // Intake roller bindings
        // firstDriver.leftTrigger().onTrue(new InstantCommand(() -> intake.rollerIn(1)));
        // firstDriver.leftTrigger().onFalse(new InstantCommand(() -> intake.rollerStop()));
        // firstDriver.leftBumper().onTrue(new InstantCommand(() -> intake.rollerOut(1)));
        // firstDriver.leftBumper().onFalse(new InstantCommand(() -> intake.rollerStop()));

        /* For Second Driver */
        secondDriver.leftTrigger().onTrue(new InstantCommand(() -> intake.rollerIn(0.5)));
        secondDriver.leftTrigger().onFalse(new InstantCommand(() -> intake.rollerStop()));
        secondDriver.leftBumper().onTrue(new InstantCommand(() -> intake.rollerOut(0.5)));
        secondDriver.leftBumper().onFalse(new InstantCommand(() -> intake.rollerStop()));

        secondDriver.rightTrigger().onTrue(new InstantCommand(() -> intake.armDown(0.1)));
        secondDriver.rightTrigger().onFalse(new InstantCommand(() -> intake.armStop()));
        secondDriver.rightBumper().onTrue(new InstantCommand(() -> intake.armUp(0.1)));
        secondDriver.rightBumper().onFalse(new InstantCommand(() -> intake.armStop()));

        // Toggle Shooter (scoring)
        secondDriver.x().toggleOnTrue(new InstantCommand(() -> shooter.setUpperPower(0.8)));
        // Toggle Shooter (passing)
        secondDriver.y().toggleOnTrue(new InstantCommand(() -> shooter.setUpperPower(0.3)));

        secondDriver.a().onTrue(new ArmToSetpoint(intake, 10));

        secondDriver.b().onTrue(new InstantCommand(() -> shooter.stopUpper()));

        // secondDriver.povUp().onTrue(new InstantCommand( () -> climber.set(0.5)));
        // secondDriver.povUp().onFalse(new InstantCommand( () -> climber.stop()));
        // secondDriver.povDown().onTrue(new InstantCommand( () -> climber.set(-0.5)));
        // secondDriver.povDown().onFalse(new InstantCommand( () -> climber.stop()));
        
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
        
        
        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
