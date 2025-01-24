package frc.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.RobotIdleMode;
import frc.robot.subsystems.drive.Drive;

public class RobotModeTo extends InstantCommand {

  private final Drive drive;
  private final RobotIdleMode mode;

  public RobotModeTo(RobotIdleMode mode, Drive drive) {
    this.mode = mode;
    this.drive = drive;
  }

  @Override
  public void initialize() {
    /*switch (mode) {
        case COAST:
        drive.setBrakeMode(false);
            break;
        default:
            drive.setBrakeMode(true);
            break;
    }*/
  }
}
