// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  public void periodic() {

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    Logger.recordOutput("Rotation to optimize", getAngle());

    state.optimize(getAngle());
    state.cosineScale(getAngle());

    Logger.recordOutput("Rotation optimized", state.angle);

    io.runDriveVelocity(state.speedMetersPerSecond / TunerConstants.wheelRadiusMeters);
    double setpoint = normalizeAngle(state.angle.getRotations(), getAngle().getRotations());

    io.runTurnPosition(Rotation2d.fromRotations(setpoint));
  }

  private double normalizeAngle(double targetRotations, double currentRotations) {
    double error = targetRotations - currentRotations;
    return currentRotations + MathUtil.inputModulus(error, -0.5, 0.5);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.runDriveOpenLoop(output);
    io.runTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.runDriveOpenLoop(0.0);
    io.runTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * TunerConstants.wheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * TunerConstants.wheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  public void setBrakeMode(boolean enable) {
    io.runDriveBrakeMode(enable);
    io.runTurnBrakeMode(enable);
  }
}