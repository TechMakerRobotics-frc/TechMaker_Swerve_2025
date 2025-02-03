package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;

/** Command to drive the robot to a specified pose or target with various input options. */
public class DriveTo extends Command {

  private final double timeOut;
  private PathConstraints constraints = new PathConstraints(4.5, 2.5, 4.5, 2.5);
  private Pose2d goalPose;
  private Command pathfollower;
  private Timer time = new Timer();
  private boolean isFinished = false;
  private final Drive drive;

  /**
   * Constructs a DriveTo command targeting a specific pose.
   *
   * @param goalPose the target pose to drive to.
   * @param timeOut the timeout for the command in seconds.
   */
  public DriveTo(Drive drive, Pose2d goalPose, double timeOut) {
    this.drive = drive;
    this.timeOut = timeOut;
    this.goalPose = goalPose;
  }

  /**
   * Constructs a DriveTo command targeting a translation with default rotation.
   *
   * @param goalTranslation the target translation to drive to.
   * @param timeOut the timeout for the command in seconds.
   */
  public DriveTo(Drive drive, Translation2d goalTranslation, double timeOut) {
    this.drive = drive;
    this.timeOut = timeOut;
    this.goalPose = new Pose2d(goalTranslation, new Rotation2d());
  }

  /**
   * Constructs a DriveTo command targeting a translation and a specific rotation.
   *
   * @param goalTranslation the target translation to drive to.
   * @param goalRotation the target rotation at the destination.
   * @param timeOut the timeout for the command in seconds.
   */
  public DriveTo(
      Drive drive, Translation2d goalTranslation, Rotation2d goalRotation, double timeOut) {
    this.drive = drive;
    this.timeOut = timeOut;
    this.goalPose = new Pose2d(goalTranslation, goalRotation);
  }

  /**
   * Constructs a DriveTo command targeting a specific x and y position with default rotation.
   *
   * @param x the x-coordinate of the target position.
   * @param y the y-coordinate of the target position.
   * @param timeOut the timeout for the command in seconds.
   */
  public DriveTo(Drive drive, double x, double y, double timeOut) {
    this.drive = drive;
    this.timeOut = timeOut;
    this.goalPose = new Pose2d(x, y, new Rotation2d());
  }

  /**
   * Constructs a DriveTo command targeting a specific x, y position and heading.
   *
   * @param x the x-coordinate of the target position.
   * @param y the y-coordinate of the target position.
   * @param heading the target heading at the destination (in degrees).
   * @param timeOut the timeout for the command in seconds.
   */
  public DriveTo(Drive drive, double x, double y, double heading, double timeOut) {
    this.drive = drive;
    this.timeOut = timeOut;
    this.goalPose = new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(heading)));
  }

  /**
   * Constructs a DriveTo command targeting a specific AprilTag pose from its ID.
   *
   * @param zoneManager the ZoneManager instance to retrieve the closest zone pose.
   * @param tag the ID of the AprilTag to target.
   * @param timeOut the timeout for the command in seconds.
   */
  public DriveTo(Drive drive, double x, double y, int tag, double timeOut) {
    this.drive = drive;
    AprilTagFieldLayout fTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    this.timeOut = timeOut;

    Pose2d tagPose =
        fTagFieldLayout
            .getTagPose(tag)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + tag))
            .toPose2d();

    this.goalPose =
        new Pose2d(x, y, new Rotation2d(GeomUtil.thetaToTarget(drive.getPose(), tagPose)));
  }

  @Override
  public void initialize() {
    addRequirements(drive);
    time.reset();
    time.start();
    pathfollower = AutoBuilder.pathfindToPose(goalPose, constraints);
    pathfollower.initialize();
  }

  @Override
  public void execute() {
    pathfollower.execute();
  }

  @Override
  public boolean isFinished() {
    return pathfollower == null
        || pathfollower.isFinished()
        || time.hasElapsed(timeOut)
        || isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    time.stop();
  }
}
