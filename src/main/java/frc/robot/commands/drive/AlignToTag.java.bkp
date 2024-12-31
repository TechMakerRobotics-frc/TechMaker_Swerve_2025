package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.vision.VisionManager;
import frc.robot.vision.VisionPose;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Command to align the robot using vision targets detected by PhotonVision. */
public class AlignToTag extends Command {
  private static final PIDController vXController =
      new PIDController(AlignConstants.VX_P, AlignConstants.VX_I, AlignConstants.VX_D);
  private static final PIDController vYController =
      new PIDController(AlignConstants.VY_P, AlignConstants.VY_I, AlignConstants.VY_D);
  private static final PIDController vOmegaController =
      new PIDController(
          AlignConstants.V_OMEGA_P, AlignConstants.V_OMEGA_I, AlignConstants.V_OMEGA_D);

  private final Timer timer = new Timer();
  private final VisionManager manager;
  private final VisionPose visionPose;
  private final double timeout;
  private final Drive drive;

  private Command defaultCommand;
  private boolean isFinished = false;
  private double vx, vy, omega;

  /**
   * Constructs a new AlignToTag command.
   *
   * @param drive the Drive subsystem used by this command
   * @param visionPose the VisionPose providing target and pose information
   * @param timeout the time in seconds before the command times out
   */
  public AlignToTag(Drive drive, VisionPose visionPose, double timeout) {
    this.drive = drive;
    this.visionPose = visionPose;
    this.manager = this.visionPose.getTargetManager();
    this.timeout = timeout;
  }

  @Override
  public void initialize() {
    vXController.reset();
    vXController.setSetpoint(0);
    vXController.setTolerance(3);
    vYController.reset();
    vYController.setSetpoint(2);
    vYController.setTolerance(3);
    vOmegaController.reset();
    vOmegaController.setSetpoint(180);
    vOmegaController.setTolerance(3);

    timer.reset();
    timer.start();

    defaultCommand = drive.getDefaultCommand();
    drive.removeDefaultCommand();

    if (manager.getCamera() != null) {
      manager.getCamera().setLED(VisionLEDMode.kOn);
    }
  }

  @Override
  public void execute() {
    PhotonPipelineResult pipelineResult = manager.getLatestPipeline();
    if (manager.hasTarget(pipelineResult)) {
      PhotonTrackedTarget target = manager.getBestTarget(pipelineResult);
      if (target != null) {
        vx = vXController.calculate(manager.getYaw(target));

        // Apply calculated velocities to the drivetrain
        drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, vx, drive.getRotation()));

        // Update dashboard with target and control information
        printToDashboard(target);
      } else {
        SmartDashboard.putString("Vision Error", "Nenhum alvo válido encontrado.");
      }
    } else {
      SmartDashboard.putString("Vision Error", "Nenhum alvo detectado.");
    }

    isFinished = vXController.atSetpoint();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.setDefaultCommand(defaultCommand);

    if (manager.getCamera() != null) {
      manager.getCamera().setLED(VisionLEDMode.kOff);
    }

    timer.stop();
  }

  /**
   * Prints vision targeting and PID control information to the SmartDashboard.
   *
   * @param target the best vision target detected
   */
  public void printToDashboard(PhotonTrackedTarget target) {
    SmartDashboard.putNumber("Tag Yaw", manager.getYaw(target));
    SmartDashboard.putNumber("Current Tag", manager.getTargetId(target));
    SmartDashboard.putNumber("omega", omega);
    SmartDashboard.putNumber("vx", vx);
    SmartDashboard.putNumber("vy", vy);
    SmartDashboard.putNumber("Target Angle", manager.getAngle(target));
    SmartDashboard.putNumber("Target Distance", manager.getDistance(target));
    SmartDashboard.putNumber("PID VX Error", vXController.getPositionError());
    SmartDashboard.putNumber("PID VY Error", vYController.getPositionError());
    SmartDashboard.putNumber("PID OMEGA Error", vOmegaController.getPositionError());
    SmartDashboard.putBoolean("PID VX At setpoint", vXController.atSetpoint());
    SmartDashboard.putBoolean("PID VY At setpoint", vYController.atSetpoint());
    SmartDashboard.putBoolean("PID OMEGA At setpoint", vOmegaController.atSetpoint());
  }
}
