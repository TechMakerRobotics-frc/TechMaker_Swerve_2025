package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class IntakeConstants {

    // Sensors
    public static final int INSIDE_SENSOR_CHANNEL = 0;
    public static final int OUTSIDE_SENSOR_CHANNEL = 1;

    // Solenoids
    public static final int SOLENOID_MODULE_CAN_ID = 19;
    public static final PneumaticsModuleType SOLENOID_MODULE_TYPE = PneumaticsModuleType.REVPH;
    public static final int SOLENOID_FORWARD_CHANNEL = 6;
    public static final int SOLENOID_REVERSE_CHANNEL = 7;
}
