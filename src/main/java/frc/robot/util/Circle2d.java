package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.ZoneLocates.Zones;

public class Circle2d {
  private final Translation2d center;
  private final double radius;
  private final Zones zone;

  public Circle2d(double centerX, double centerY, double radius, Zones zone) {
    this.center = new Translation2d(centerX, centerY);
    this.radius = radius;
    this.zone = zone;
  }

  public Circle2d(Translation2d translation, double radius, Zones zone) {
    this.center = translation;
    this.radius = radius;
    this.zone = zone;
  }

  public Translation2d getCenter() {
    return center;
  }

  public double getX() {
    return center.getX();
  }

  public double getY() {
    return center.getY();
  }

  public double getRadius() {
    return radius;
  }

  public Zones getZone() {
    return zone;
  }

  public double getRadiusInInches() {
    return Units.metersToInches(radius);
  }

  public boolean containsPoint(Translation2d point) {
    return center.getDistance(point) <= radius;
  }
}
