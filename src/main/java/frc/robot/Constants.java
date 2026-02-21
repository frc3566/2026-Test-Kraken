// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.math.util.Units.inchesToMeters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Motors{
    public static final int ShooterHigh = 32;
    public static final int ShooterLow = 22;
    public static final int IntakeRoller = 40;
    public static final int IntakeArm = 41;
    public static final int Agitator = 23;
  }

  public static class Arm{
    public static final double GearRatio = 64.0;
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
     */
    public static final Transform3d robotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(14.5),
            Units.inchesToMeters(0),
            Units.inchesToMeters(10)
        ),
        new Rotation3d(0, 0, 0)
        );
    }

    public static final Transform2d robotToCamera2d = new Transform2d(
    new Translation2d(inchesToMeters(14.5), 0.0), 
    new Rotation2d(0.0)
    );
}
