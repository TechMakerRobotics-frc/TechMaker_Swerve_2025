package frc.robot.subsystems.drive;

import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.Circle2d;

public class DriveConstants {

    public static class ZoneLocates {
        public static final Circle2d blueLeftStation = new Circle2d(1.100, 7.124, 2.6, Zones.BLUE_LEFT_STATION);
        public static final Circle2d blueRightStation = new Circle2d(1.100, 0.974, 2.6, Zones.BLUE_RIGHT_STATION);
        public static final Circle2d redLeftStation = new Circle2d(16.697198, 0.974, 2.6, Zones.RED_LEFT_STATION);
        public static final Circle2d redRightStation = new Circle2d(16.411, 6.968, 2.6, Zones.RED_RIGHT_STATION);
        public static final Circle2d blueProcessor = new Circle2d(6.341, 0.471, 1, Zones.BLUE_PROCESSOR);
        public static final Circle2d redProcessor = new Circle2d(11.540, 7.651, 1, Zones.RED_PROCESSOR);

        // Lista contendo todas as zonas
        public static final List<Circle2d> zones = List.of(
            blueLeftStation,
            blueRightStation,
            redLeftStation,
            redRightStation,
            blueProcessor,
            redProcessor
        );

        public static enum Zones {
            NOT_ZONE,
            BLUE_LEFT_STATION,
            BLUE_RIGHT_STATION,
            RED_LEFT_STATION,
            RED_RIGHT_STATION,
            BLUE_PROCESSOR,
            RED_PROCESSOR,
        }
    }

  public static final class ModuleConstants {
    // Gear ratios for SDS MK4i L3, adjust as necessary
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    public static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs().withKP(0.03).withKI(0).withKD(0).withKS(0).withKV(0.124);

    // Modules:
    // Front Left
    public static final int TALON_FL = 1;
    public static final int SPARK_FL = 2;
    public static final int CANCODER_FL = 3;
    public static final double ENCODER_OFFSET_FL = (-0.408203);

    // Front Right
    public static final int TALON_FR = 4;
    public static final int SPARK_FR = 5;
    public static final int CANCODER_FR = 6;
    public static final double ENCODER_OFFSET_FR = (0.325195);

    // Back Left
    public static final int TALON_BL = 7;
    public static final int SPARK_BL = 8;
    public static final int CANCODER_BL = 9;
    public static final double ENCODER_OFFSET_BL = (0.389404);

    // Back Right
    public static final int TALON_BR = 10;
    public static final int SPARK_BR = 11;
    public static final int CANCODER_BR = 12;
    public static final double ENCODER_OFFSET_BR = (0.374023);

    public static final String CANBUS = "CANivore";

    public static final boolean TURN_MOTOR_INVERTED = true;

    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;

    public static final double ODOMETRY_UPDATE_FREQUENCY = 100.0;
    public static final double OTHER_SIGNALS_UPDATE_FREQUENCY = 50.0;

    public static final int CAN_TIMEOUT_MS = 250;
    public static final int SMART_CURRENT_LIMIT = 30;
    public static final double VOLTAGE_COMPENSATION = 10.0;

    public static final double INITIAL_ENCODER_POSITION = 0.0;
    public static final int ENCODER_MEASUREMENT_PERIOD_MS = 10;
    public static final int ENCODER_AVERAGE_DEPTH = 2;

    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(ENCODER_OFFSET_FL);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(ENCODER_OFFSET_FR);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(ENCODER_OFFSET_BL);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(ENCODER_OFFSET_BR);

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
  }
}
