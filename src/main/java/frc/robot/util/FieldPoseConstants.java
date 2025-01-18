package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldPoseConstants {

    public static class ReefPoses {
        // Blue Reef Poses
        public static final Pose2d A1_BLUE = new Pose2d(6.066, 4.187, new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d B1_BLUE = new Pose2d(6.066, 3.875, new Rotation2d(Units.degreesToRadians(180)));

        public static final Pose2d A2_BLUE = new Pose2d(5.263, 3.012, new Rotation2d(Units.degreesToRadians(120)));
        public static final Pose2d B2_BLUE = new Pose2d(4.963, 2.832, new Rotation2d(Units.degreesToRadians(120)));

        public static final Pose2d A3_BLUE = new Pose2d(3.992, 2.856, new Rotation2d(Units.degreesToRadians(60)));
        public static final Pose2d B3_BLUE = new Pose2d(3.728, 3.036, new Rotation2d(Units.degreesToRadians(60)));

        public static final Pose2d A4_BLUE = new Pose2d(3.237, 3.863, new Rotation2d(Units.degreesToRadians(0)));
        public static final Pose2d B4_BLUE = new Pose2d(3.237, 4.190, new Rotation2d(Units.degreesToRadians(0)));

        public static final Pose2d A5_BLUE = new Pose2d(3.728, 5, new Rotation2d(Units.degreesToRadians(-60)));
        public static final Pose2d B5_BLUE = new Pose2d(4, 5.182, new Rotation2d(Units.degreesToRadians(-60)));

        public static final Pose2d A6_BLUE = new Pose2d(4.963, 5.170, new Rotation2d(Units.degreesToRadians(-120)));
        public static final Pose2d B6_BLUE = new Pose2d(5.251, 5, new Rotation2d(Units.degreesToRadians(-120)));    
        
        //Red Reef Poses
        public static final Pose2d A1_RED = new Pose2d(11.844, 3.875, new Rotation2d(Units.degreesToRadians(0)));
        public static final Pose2d B1_RED = new Pose2d(11.844, 4.187, new Rotation2d(Units.degreesToRadians(0)));

        public static final Pose2d A2_RED = new Pose2d(12.299, 5.014, new Rotation2d(Units.degreesToRadians(-60)));
        public static final Pose2d B2_RED = new Pose2d(12.599, 5.170, new Rotation2d(Units.degreesToRadians(-60)));

        public static final Pose2d A3_RED = new Pose2d(13.534, 5.170, new Rotation2d(Units.degreesToRadians(-120)));
        public static final Pose2d B3_RED = new Pose2d(13.822, 5, new Rotation2d(Units.degreesToRadians(-120)));

        public static final Pose2d A4_RED = new Pose2d(14.301, 4.175, new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d B4_RED = new Pose2d(14.301, 3.863, new Rotation2d(Units.degreesToRadians(180)));

        public static final Pose2d A5_RED = new Pose2d(13.810, 3.060, new Rotation2d(Units.degreesToRadians(120)));
        public static final Pose2d B5_RED = new Pose2d(13.534, 2.856, new Rotation2d(Units.degreesToRadians(120)));

        public static final Pose2d A6_RED = new Pose2d(12.587, 2.856, new Rotation2d(Units.degreesToRadians(60)));
        public static final Pose2d B6_RED = new Pose2d(12.299, 3.024, new Rotation2d(Units.degreesToRadians(60)));  
    }

    public static class CoralStationPoses {
        public static final Pose2d LEFT_BLUE = new Pose2d(1.100, 4.918, new Rotation2d(Units.degreesToRadians(-53.631)));
        public static final Pose2d RIGHT_BLUE = new Pose2d(1.100, 0.974, new Rotation2d(Units.degreesToRadians(53.631)));

        public static final Pose2d LEFT_RED = new Pose2d(16.411, 0.974, new Rotation2d(Units.degreesToRadians(125.116))); 
        public static final Pose2d RIGHT_RED = new Pose2d(16.411, 6.968, new Rotation2d(Units.degreesToRadians(-125.116))); 
    }

    public static class ProcessorPoses {
        public static final Pose2d BLUE = new Pose2d(6.341, 0.471, new Rotation2d(Units.degreesToRadians(90)));
        public static final Pose2d RED = new Pose2d(11.540, 7.651, new Rotation2d(Units.degreesToRadians(-90)));
    }
}
