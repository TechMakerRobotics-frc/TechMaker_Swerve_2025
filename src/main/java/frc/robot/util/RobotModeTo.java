package frc.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

public class RobotModeTo extends InstantCommand {

  private final Drive drive;
  private String mode;

  public RobotModeTo(String mode, Drive drive) {
    this.mode = mode;
    this.drive = drive;
  }

  @Override
  public void initialize() {
    if (mode.equalsIgnoreCase("Coast")) {
      drive.setBrakeMode(false);
    } else if (mode.equalsIgnoreCase("Brake") || mode.equalsIgnoreCase("Break")) {
      drive.setBrakeMode(true);
    }
  }
}
