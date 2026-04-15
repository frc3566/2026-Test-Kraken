// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Swerve {
    public static final double MaxSpeed = 3.79;
  }
  public static class Motors{
    public static final int ShooterLeft = 50;
    public static final int ShooterRight = 51;
    public static final int IndexerLeft = 52;
    public static final int IndexerRight = 53;

    public static final int IntakeRoller = 40;
    public static final int IntakeArmCanCoder = 41;
    public static final int IntakeArmLeader = 42;

  }

  public static class Arm{
    public static final double GearRatio = 45.0;
    public static final double UpSetpointRotations = 0.377;
    public static final double DownSetpointRotations = 0.00;
    public static final double PovDownSetpointRotations = 0;

    public static final double DefaultToleranceRotations = 0.02;
    public static final double PovUpToleranceRotations = 0.05;
    public static final double PovDownToleranceRotations = 0.05;

    public static final double ManualPower = 0.15;

    public static final double UpAndDownIntervalSeconds = 0.75;
    public static final double UpAndDownHighRotations = 0.25;
    public static final double UpAndDownLowRotations = 0.00;

    public static final double ArmToSetpointTimeoutSeconds = 10.0;
    public static final double ArmSwitchTimeoutSeconds = 1.5;

    public static final double ForwardSoftLimitRotations = 0.45;
    public static final double ReverseSoftLimitRotations = -0.05;
  }

  public static class Climber{
    public static final double topSetPoint = 0.0;
    public static final double bottomSetPoint = -180.0;
  }
  
  public static class AprilTags{
    public static final int RED_STAGE_SIDE = 9;
    public static final int RED_STAGE_CENTER = 10;

    public static final int BLUE_STAGE_SIDE = 26;
    public static final int BLUE_STAGE_CENTER = 25;

  }


  public static class Vision {
    public static final double xWidth = Units.inchesToMeters(32.5);
    public static final double yWidth = Units.inchesToMeters(29);

    /** 
     * For simulation only.
     * Get properties from https://photonvision.local:5800 
     */
    public static final SimCameraProperties cameraProperties = new SimCameraProperties();
    static {
        cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(74.10));
        cameraProperties.setCalibError(0.68, 0.08);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);
    }

    /**
     * Reference coordinate system: 
     * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html 
    //  */
    // public static final Transform3d robotToCamera = new Transform3d(
    //     new Translation3d(
    //         Units.inchesToMeters(14.337),
    //         Units.inchesToMeters(0),
    //         Units.inchesToMeters(10.486)
    //     ),
    //     new Rotation3d(0, -0.1745, 0)
    //     );

    // public static final Transform2d robotToCamera2d = new Transform2d(
    //   new Translation2d(inchesToMeters(14.337), 0.0), 
    //   new Rotation2d(0.0)
    // );
    }


}
