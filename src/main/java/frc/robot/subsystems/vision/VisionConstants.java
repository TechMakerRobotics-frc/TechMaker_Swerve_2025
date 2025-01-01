// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    public static final String FL_CAM_NAME = "FLCam";
    public static final double FL_CAM_X = 0.265;
    public static final double FL_CAM_Y = 0.235;
    public static final double FL_CAM_Z = 0.20;
    public static final double FL_CAM_ROLL = 0.0;
    public static final double FL_CAM_PITCH = -30;
    public static final double FL_CAM_YAW = 25;

    public static final String FR_CAM_NAME = "FRCam";
    public static final double FR_CAM_X = 0.265;
    public static final double FR_CAM_Y = -0.235;
    public static final double FR_CAM_Z = 0.20;
    public static final double FR_CAM_ROLL = 0.0;
    public static final double FR_CAM_PITCH = -30;
    public static final double FR_CAM_YAW = -25;

    public static final String LIMELIGHT_NAME = "Limelight";
    public static final double LIMELIGHT_X = 0.155;
    public static final double LIMELIGHT_Y = 0;
    public static final double LIMELIGHT_Z = 0.545;
    public static final double LIMELIGHT_ROLL = 0;
    public static final double LIMELIGHT_PITCH = -25;
    public static final double LIMELIGHT_YAW = 180;

    public static final Transform3d ROBOT_TO_FL_CAM = new Transform3d(
            new Translation3d(FL_CAM_X, FL_CAM_Y, FL_CAM_Z),
            new Rotation3d(
                    Units.degreesToRadians(FL_CAM_ROLL),
                    Units.degreesToRadians(FL_CAM_PITCH),
                    Units.degreesToRadians(FL_CAM_YAW)));

    public static final Transform3d ROBOT_TO_FR_CAM = new Transform3d(
            new Translation3d(FR_CAM_X, FR_CAM_Y, FR_CAM_Z),
            new Rotation3d(
                    Units.degreesToRadians(FR_CAM_ROLL),
                    Units.degreesToRadians(FR_CAM_PITCH),
                    Units.degreesToRadians(FR_CAM_YAW)));

    public static final Transform3d ROBOT_TO_LIMELIGHT = new Transform3d(
            new Translation3d(LIMELIGHT_X, LIMELIGHT_Y, LIMELIGHT_Z),
            new Rotation3d(
                    Units.degreesToRadians(LIMELIGHT_ROLL),
                    Units.degreesToRadians(LIMELIGHT_PITCH),
                    Units.degreesToRadians(LIMELIGHT_YAW)));

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
