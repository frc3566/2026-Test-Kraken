// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Shuffle;
import frc.robot.commands.intake.ArmSwitch;
import frc.robot.commands.intake.ArmToSetpoint;
import frc.robot.commands.intake.ArmUpAndDown;
import frc.robot.commands.shooter.AutoPrime;
import frc.robot.commands.shooter.PrimeAndShoot;
import frc.robot.commands.shooter.PrimeAndShootFixed;
import frc.robot.commands.vision.AutoPower;
import frc.robot.commands.vision.DriveToPose;
import frc.robot.commands.vision.DriveToPoseNew;
import frc.robot.commands.vision.HeadToAngle;
import frc.robot.commands.vision.TagUtil;
import frc.robot.commands.vision.TurnToHub;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private boolean enableDrive = true;
    private double autonomousPower = 60;
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
    public final Vision vision = new Vision(drivetrain::getPose);
    public final Climber climber = new Climber();
    // public final Climber climber = new Climber();

    public  SendableChooser<Command> firstChooser;
    // public  SendableChooser<Command> secondChooser;
    // public  SendableChooser<Command> thirdChooser;

    public RobotContainer() {
        if(!Utils.isSimulation()){
            // configureCamera();
        }

        
        configureAutoCommand();
        configureAutoChooser();
        configureBindings();

    }

    private void configureCamera() {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(15);
    }

    private void configureAutoChooser() {
        firstChooser = AutoBuilder.buildAutoChooser();
        // secondChooser = AutoBuilder.buildAutoChooser();
        // thirdChooser = AutoBuilder.buildAutoChooser();

        Shuffleboard.getTab("Autonomous").add("First Auto", firstChooser);
        // Shuffleboard.getTab("Autonomous").add("Second Auto", secondChooser);
        // Shuffleboard.getTab("Autonomous").add("Third Auto", thirdChooser);
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();


        /* SysID stuffs */
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // secondDriver.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // secondDriver.povDown().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // secondDriver.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // secondDriver.povRight().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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

        // vision.setDefaultCommand(new UpdateVisionPoseEstimate(vision, drivetrain));
            
        // firstDriver.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
     
        // firstDriver.y().toggleOnTrue(new LogTargetDistance(vision));

        // Left trigger = always turn to hub
        firstDriver.leftTrigger().whileTrue(new TurnToHub(drivetrain, 
            () -> -firstDriver.getLeftY() * MaxSpeed, 
            () -> -firstDriver.getLeftX() * MaxSpeed,
            MaxSpeed,
            MaxAngularRate));

        // Right trigger = intake facing right

        firstDriver.rightTrigger().whileTrue(new HeadToAngle(
            90,
            drivetrain, 
            () -> -firstDriver.getLeftY() * MaxSpeed, 
            () -> -firstDriver.getLeftX() * MaxSpeed,
            MaxSpeed,
            MaxAngularRate));


        firstDriver.y().whileTrue(new HeadToAngle(
            -90,
            drivetrain, 
            () -> -firstDriver.getLeftY() * MaxSpeed, 
            () -> -firstDriver.getLeftX() * MaxSpeed,
            MaxSpeed,
            MaxAngularRate));


        // b = only straight forward, no left or right
        firstDriver.b().whileTrue(
             drivetrain.applyRequest(() ->
                    drive.withVelocityX(-firstDriver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(0) // Drive left with negative X (left)
                        .withRotationalRate(-firstDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        // x = turn back
        firstDriver.x().whileTrue(new HeadToAngle(
            -180,
            drivetrain, 
            () -> -firstDriver.getLeftY() * MaxSpeed, 
            () -> -firstDriver.getLeftX() * MaxSpeed,
            MaxSpeed,
            MaxAngularRate)
        );

        firstDriver.leftBumper().whileTrue(new DriveToPose(drivetrain, vision, () -> TagUtil.getLeftTrenchPose()));
        firstDriver.rightBumper().whileTrue(new DriveToPose(drivetrain, vision, () -> TagUtil.getRightTrenchPose()));
        firstDriver.povLeft().whileTrue(new DriveToPoseNew(vision, () -> TagUtil.getLeftTrenchPose()));
        firstDriver.povRight().whileTrue(new DriveToPoseNew(vision, () -> TagUtil.getRightTrenchPose()));
        firstDriver.povUp().onTrue(new Shuffle(drivetrain));
        
       

        firstDriver.a().whileTrue(
                Commands.startEnd(
                    () -> shooter.setLowerPower(60),
                    () -> shooter.stopLower(),
                    shooter)
        );

        /* For Second Driver */
    secondDriver.leftTrigger().onTrue(new InstantCommand(() -> intake.rollerIn(35)));
    secondDriver.leftTrigger().onFalse(new InstantCommand(() -> intake.stopRoller()));
    secondDriver.rightTrigger().onTrue(new InstantCommand(() -> intake.rollerOut(35)));
    secondDriver.rightTrigger().onFalse(new InstantCommand(() -> intake.stopRoller()));


    // Hold left bumper to move arm up and down repeatedly for intaking multiple balls.
    secondDriver.leftBumper().whileTrue(new ArmUpAndDown(intake));
    
    // Arm encoder resets on RB when arm is up
    secondDriver.rightBumper().onTrue(new InstantCommand(() -> intake.setArmEncoderPosition(0.0)));

        // secondDriver.rightTrigger().onTrue(new InstantCommand(() -> intake.armUp(0.2)));
        // secondDriver.rightTrigger().onFalse(new InstantCommand(() -> intake.stopArm()));
        // secondDriver.rightBumper().onTrue(new InstantCommand(() -> intake.armDown(0.2)));
        // secondDriver.rightBumper().onFalse(new InstantCommand(() -> intake.stopArm()));

        // Scoring 
        secondDriver.x().onTrue(new InstantCommand(() -> shooter.setUpperPower(43)));
        
        // Neutral Passing 
        secondDriver.b().onTrue(new InstantCommand(() -> shooter.setUpperPower(70)));

        // Auto Power
        secondDriver.y().onTrue(new AutoPower(shooter, () -> drivetrain.getState().Pose, TagUtil::getHubFrontCenterTagTranslation));

        // Stop shooter and interrupt auto power
        secondDriver.a().onTrue(shooter.runOnce(() -> shooter.stopUpper()));

        
        secondDriver.povUp().whileTrue(new ArmToSetpoint(intake, 0.0, 0.05));
        secondDriver.povDown().whileTrue(new ArmToSetpoint(intake, 0.4, 0.01));

        // secondDriver.povLeft().onTrue(new InstantCommand(() -> intake.resetArmPosition(true)));
        // secondDriver.povRight().onTrue(new InstantCommand(() -> intake.resetArmPosition(false)));

        secondDriver.povLeft().onTrue(new InstantCommand(() -> intake.setArmPower(0.1)));
        secondDriver.povLeft().onFalse(new InstantCommand(() -> intake.stopArm()));
        secondDriver.povRight().onTrue(new InstantCommand(() -> intake.setArmPower(-0.1)));
        secondDriver.povRight().onFalse(new InstantCommand(() -> intake.stopArm()));

        // Quick back/forth jostle to shift balls forward in the hopper
        secondDriver.start().onTrue(new Shuffle(drivetrain));


        // secondDriver.povRight().onTrue(new InstantCommand( () -> shooter.addTestSpeed(0.01)));
        // secondDriver.povLeft().onTrue(new InstantCommand( () -> shooter.addTestSpeed(-0.01)));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        // Wrap each chooser selection in a ProxyCommand so a fresh command instance is
        // created every time auto runs. Without this, reusing the same Command object
        // in a SequentialCommandGroup crashes on the second auto attempt because
        // WPILib commands can only be composed/scheduled once. We don't want this
        // because during testing we want to run auto many times in a row without restarting the robot code.

        Command start = firstChooser.getSelected();
        // Command second = secondChooser.getSelected();
        // Command third = thirdChooser.getSelected();

        if (start != null) autoCommand.addCommands(new ProxyCommand(start));
        // if (second != null) autoCommand.addCommands(new ProxyCommand(second));
        // if (third != null) autoCommand.addCommands(new ProxyCommand(third));

        return autoCommand;
    }

    private void configureAutoCommand() {

        // Prime the shooter while driving
        NamedCommands.registerCommand(
            "AutoPrime",
            new AutoPrime(shooter, intake) // Make it long enough so keeps running until deadline group?
        );

        // Generic Prime and Shoot.
        // We want to wait until the robot is stead, so 0.5 second for another priming
        NamedCommands.registerCommand(
            "PrimeAndShoot",
            new PrimeAndShoot(shooter, intake, () -> drivetrain.getState().Pose, TagUtil::getHubFrontCenterTagTranslation, 0.5, 4)
        );


        NamedCommands.registerCommand(
            "LeftPrimeAndShoot",
            new PrimeAndShootFixed(shooter, intake, autonomousPower, 1, 4)
        );

        // For some reason right side need more power.
        // Could be due to inaccurate field, use PrimeAndShoot if things are not right
        NamedCommands.registerCommand(
            "RightPrimeAndShoot",
            new PrimeAndShootFixed(shooter, intake, autonomousPower, 1, 4)
        );

        NamedCommands.registerCommand(
            "RightPrimeAndShootLong",
            new PrimeAndShootFixed(shooter, intake, autonomousPower, 1, 7)
        );

        NamedCommands.registerCommand(
            "CenterPrimeAndShoot",
            new PrimeAndShootFixed(shooter, intake, 54, 1, 4)
        );

        NamedCommands.registerCommand(
            "ArmDown",
            new ArmToSetpoint(intake, 0.4, 0.01)
        );

        NamedCommands.registerCommand(
            "Shuffle",
            new Shuffle(drivetrain)
        );
    }

}
