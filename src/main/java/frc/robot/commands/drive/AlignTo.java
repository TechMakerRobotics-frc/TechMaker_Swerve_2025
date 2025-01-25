package frc.robot.commands.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AlignTo extends Command {
  private final Drive drive;
  private final Pose2d targetPose;
  private PIDController thetaController;
  private final Timer time = new Timer();
  private final double timeOut;
  private LoggedNetworkNumber tP = new LoggedNetworkNumber("/Tuning/P", 1);
  private LoggedNetworkNumber tI = new LoggedNetworkNumber("/Tuning/I", 0);
  private LoggedNetworkNumber tD = new LoggedNetworkNumber("/Tuning/D", 0);

  public AlignTo(Drive drive, double x, double y, double timeOut) {
    this.drive = drive;
    this.targetPose = new Pose2d(x, y, new Rotation2d());
    this.timeOut = timeOut;
    addRequirements(drive);
  }

  public AlignTo(Drive drive, Pose2d pose, double timeOut) {
    this.drive = drive;
    this.targetPose = pose;
    this.timeOut = timeOut;
    addRequirements(drive);
  }

  public AlignTo(Drive drive, Translation2d translation, double timeOut) {
    this.drive = drive;
    this.targetPose = new Pose2d(translation, new Rotation2d());
    this.timeOut = timeOut;
    addRequirements(drive);
  }

  public AlignTo(Drive drive, int tag, double timeOut) {
    this.drive = drive;
    AprilTagFieldLayout fTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    this.targetPose = fTagFieldLayout.getTagPose(tag).get().toPose2d();
    this.timeOut = timeOut;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    thetaController = new PIDController(tP.get(), tI.get(), tD.get());
    thetaController.reset();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(0.05);
    time.reset();
    time.start();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Rotation2d currentRotation = currentPose.getRotation();

    double rotationSpeed =
        thetaController.calculate(
            currentRotation.getRadians(), GeomUtil.thetaToTarget(currentPose, targetPose));

    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotationSpeed);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());

    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint() || time.hasElapsed(timeOut);
  }
}
