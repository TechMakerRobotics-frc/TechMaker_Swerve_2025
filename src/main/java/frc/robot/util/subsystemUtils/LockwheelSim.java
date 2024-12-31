// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.subsystemUtils;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Represents a simulated Lockwheel mechanism. */
public class LockwheelSim extends LinearSystemSim<N1, N1, N1> {
    // Gearbox for the Lockwheel.
    private final DCMotor m_gearbox;

    // The gearing from the motors to the output.
    private final double m_gearing;

    // The moment of inertia for the Lockwheel mechanism.
    private final double m_jKgMetersSquared;

    /**
     * Creates a simulated Lockwheel mechanism.
     *
     * @param plant The linear system that represents the Lockwheel. Use either
     *     {@link LinearSystemId#createLockwheelSystem(DCMotor, double, double)} if using physical constants or
     *     {@link LinearSystemId#identifyVelocitySystem(double, double)} if using system characterization.
     * @param gearbox The type of and number of motors in the Lockwheel gearbox.
     * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no noise is desired. If
     *     present must have 1 element for velocity.
     */
    public LockwheelSim(LinearSystem<N1, N1, N1> plant, DCMotor gearbox, double... measurementStdDevs) {
        super(plant, measurementStdDevs);
        m_gearbox = gearbox;

        // By theorem 6.10.1 of https://file.tavsys.net/control/controls-engineering-in-frc.pdf,
        // the Lockwheel state-space model is:
        //
        //   dx/dt = -G²Kₜ/(KᵥRJ)x + (GKₜ)/(RJ)u
        //   A = -G²Kₜ/(KᵥRJ)
        //   B = GKₜ/(RJ)
        //
        // Solve for G.
        //
        //   A/B = -G/Kᵥ
        //   G = -KᵥA/B
        //
        // Solve for J.
        //
        //   B = GKₜ/(RJ)
        //   J = GKₜ/(RB)
        m_gearing = -gearbox.KvRadPerSecPerVolt * plant.getA(0, 0) / plant.getB(0, 0);
        m_jKgMetersSquared = m_gearing * gearbox.KtNMPerAmp / (gearbox.rOhms * plant.getB(0, 0));
    }

    /**
     * Sets the Lockwheel's angular velocity.
     *
     * @param velocityRadPerSec The new velocity in radians per second.
     */
    public void setAngularVelocity(double velocityRadPerSec) {
        setState(VecBuilder.fill(velocityRadPerSec));
    }

    /**
     * Returns the gear ratio of the Lockwheel.
     *
     * @return the Lockwheel's gear ratio.
     */
    public double getGearing() {
        return m_gearing;
    }

    /**
     * Returns the moment of inertia in kilograms meters squared.
     *
     * @return The Lockwheel's moment of inertia.
     */
    public double getJKgMetersSquared() {
        return m_jKgMetersSquared;
    }

    /**
     * Returns the gearbox for the Lockwheel.
     *
     * @return The Lockwheel's gearbox.
     */
    public DCMotor getGearbox() {
        return m_gearbox;
    }

    /**
     * Returns the Lockwheel's velocity in Radians Per Second.
     *
     * @return The Lockwheel's velocity in Radians Per Second.
     */
    public double getAngularVelocityRadPerSec() {
        return getOutput(0);
    }

    /**
     * Returns the Lockwheel's velocity in RPM.
     *
     * @return The Lockwheel's velocity in RPM.
     */
    public double getAngularVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(getAngularVelocityRadPerSec());
    }

    /**
     * Returns the Lockwheel's velocity.
     *
     * @return The Lockwheel's velocity
     */
    public AngularVelocity getAngularVelocity() {
        return RadiansPerSecond.of(getAngularVelocityRadPerSec());
    }

    /**
     * Returns the Lockwheel's acceleration in Radians Per Second Squared.
     *
     * @return The Lockwheel's acceleration in Radians Per Second Squared.
     */
    public double getAngularAccelerationRadPerSecSq() {
        var acceleration = (m_plant.getA().times(m_x)).plus(m_plant.getB().times(m_u));
        return acceleration.get(0, 0);
    }

    /**
     * Returns the Lockwheel's acceleration.
     *
     * @return The Lockwheel's acceleration.
     */
    public AngularAcceleration getAngularAcceleration() {
        return RadiansPerSecondPerSecond.of(getAngularAccelerationRadPerSecSq());
    }

    /**
     * Returns the Lockwheel's torque in Newton-Meters.
     *
     * @return The Lockwheel's torque in Newton-Meters.
     */
    public double getTorqueNewtonMeters() {
        return getAngularAccelerationRadPerSecSq() * m_jKgMetersSquared;
    }

    /**
     * Returns the Lockwheel's current draw.
     *
     * @return The Lockwheel's current draw.
     */
    public double getCurrentDrawAmps() {
        // I = V / R - omega / (Kv * R)
        // Reductions are output over input, so a reduction of 2:1 means the motor is spinning
        // 2x faster than the Lockwheel
        return m_gearbox.getCurrent(m_x.get(0, 0) * m_gearing, m_u.get(0, 0)) * Math.signum(m_u.get(0, 0));
    }

    /**
     * Gets the input voltage for the Lockwheel.
     *
     * @return The Lockwheel's input voltage.
     */
    public double getInputVoltage() {
        return getInput(0);
    }

    /**
     * Sets the input voltage for the Lockwheel.
     *
     * @param volts The input voltage.
     */
    public void setInputVoltage(double volts) {
        setInput(volts);
        clampInput(RobotController.getBatteryVoltage());
    }
}
