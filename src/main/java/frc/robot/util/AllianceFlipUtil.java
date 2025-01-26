// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.ModuleForce;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.VehicleState;

public class AllianceFlipUtil {
  public static double fieldWidth = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);
  public static double fieldLength = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

  public static double applyX(double x) {
    return shouldFlip() ? fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static VehicleState apply(VehicleState state) {
    return shouldFlip()
        ? VehicleState.newBuilder()
            .setX(applyX(state.getX()))
            .setY(applyY(state.getY()))
            .setTheta(apply(Rotation2d.fromRadians(state.getTheta())).getRadians())
            .setVx(-state.getVx())
            .setVy(-state.getVy())
            .setOmega(state.getOmega())
            .addAllModuleForces(
                state.getModuleForcesList().stream()
                    .map(
                        forces ->
                            ModuleForce.newBuilder()
                                .setFx(-forces.getFx())
                                .setFy(-forces.getFy())
                                .build())
                    .toList())
            .build()
        : state;
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
