// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;


    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        m_robotContainer.vision.updatePoseEstimation(m_robotContainer.drivetrain);
        putSubsystemTelemetry();
        SmartDashboard.putNumber("Robot Velocity X (m/s)", m_robotContainer.drivetrain.getState().Speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Robot Velocity Y (m/s)", m_robotContainer.drivetrain.getState().Speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Robot Angular Rate (rads)", m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond);
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        SmartDashboard.putNumber("Auto Time", DriverStation.getMatchTime());
        m_robotContainer.drivetrain.seedFieldCentric();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putNumber("Auto Time", DriverStation.getMatchTime());
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        setTeleopShifts();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        setTeleopShifts();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        putSubsystemTelemetry();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
    }

    // public void setFieldRobotPose(){
    //     // var pose = m_robotContainer.drivetrain.getState().Pose;
    //     // var pose = m_robotContainer.vision.getEstimatedGlobalPose();
    //     // var wrappedRotation = Rotation2d.fromRadians(
    //     //     Math.IEEEremainder(pose.getRotation().getRadians(), 2 * Math.PI)
    //     // );

    //     // field.setRobotPose(
    //     //     new Pose2d(pose.getTranslation(), wrappedRotation)
    //     // );
    //     SmartDashboard.putData(field);
    // }

    public void setTeleopShifts(){
        String shift = "";
        double shiftTime = 0;
        double currentTime =DriverStation.getMatchTime();
        if(currentTime > 130){
            shift = "TRANSITION SHIFT";
            shiftTime = currentTime-130;

        } else if(currentTime > 105){
            shift = "SHIFT 1";
            shiftTime = currentTime-105;
        } else if(currentTime > 80){
            shift = "SHIFT 2";
            shiftTime = currentTime-80;
        } else if(currentTime > 55){
            shift = "SHIFT 3";
            shiftTime = currentTime-55;
        }
        else if(currentTime > 30){
            shift = "SHIFT 4";
            shiftTime = currentTime-30;
        } else {
            shift = "END GAME";
            shiftTime = currentTime;
        }
         SmartDashboard.putString("Teleop Shift", shift);
         SmartDashboard.putNumber("Teleop Shift Time", shiftTime);
    }

    public void putSubsystemTelemetry() {
        double flywheelPower = m_robotContainer.shooter.upperMotor.get();
        double feederPower = m_robotContainer.shooter.lowerMotor.get();
        double armPower = m_robotContainer.intake.armMotor.get();
        double rollerPower = m_robotContainer.intake.rollerMotor.get();
        double armPosition = m_robotContainer.intake.armMotor.getPosition().getValueAsDouble() / Constants.Arm.GearRatio * 360; // Convert to output shaft position

        SmartDashboard.putNumber("Shooter Flywheel Power", flywheelPower);
        SmartDashboard.putNumber("Shooter Feeder Power", feederPower);
        SmartDashboard.putNumber("Intake Arm Power", armPower);
        SmartDashboard.putNumber("Intake Roller Power", rollerPower);
        SmartDashboard.putNumber("Intake Arm Position", armPosition);
        SmartDashboard.putNumber("PID/Shooter RPM", m_robotContainer.shooter.getUpperVelocity());
        
    }
}
