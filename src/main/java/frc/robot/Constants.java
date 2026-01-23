// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// ANGLES FOR AUTO:

//ELEVATOR HEIGHT AT BARS: 0.5, IRL: TBD
//ELEVATOR HEIGHT AT INTAKE STATION: 0.4?(closer to 0.75), IRL: TBD
//PIVOT ANGLE AT INTAKE STATION:56.2 , IRL: -24
// L2 angle 112, elevator 1.19
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static final int LEFT_MOTOR=8;
    public static final int RIGHT_MOTOR=9;
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
            Units.inchesToMeters(13.5)
        ),
        new Rotation3d(0, 0, 0)
        );
    }
}
