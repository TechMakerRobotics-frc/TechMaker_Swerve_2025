package frc.robot.interfaces.Motor;

import static frc.robot.util.subsystemUtils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOTalonFX implements MotorIO {

    protected final TalonFX motor;
    protected final VoltageOut voltageRequest = new VoltageOut(0);
    protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    protected final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Inputs from drive motor
    protected StatusSignal<Angle> motorPosition;
    protected StatusSignal<AngularVelocity> motorVelocity;
    protected StatusSignal<Voltage> motorAppliedVolts;
    protected StatusSignal<Current> motorCurrent;

    private ClosedLoopOutputType motorClosedLoopOutput;

    protected final TalonFXConfiguration driveConfig;

    public MotorIOTalonFX(int id, String CANBusName, NeutralModeValue neutral, Slot0Configs motorGains, 
                          double SlipCurrent, boolean StatorCurrentLimit, InvertedValue inverted,ClosedLoopOutputType motorClosedLoopOutput) {
        motor = new TalonFX(id, CANBusName);
        // Configure drive motor
        driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = neutral;
        driveConfig.Slot0 = motorGains;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = StatorCurrentLimit;
        driveConfig.MotorOutput.Inverted = inverted;
        this.motorClosedLoopOutput = motorClosedLoopOutput;
        tryUntilOk(5, () -> motor.getConfigurator().apply(driveConfig, 0.25));
        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        motorAppliedVolts = motor.getMotorVoltage();
        motorCurrent = motor.getStatorCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(100.0,  motorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVelocity, motorAppliedVolts, motorCurrent);

    }

    /** @param inputs */
    @Override
    public void updateInputs(MotorIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

        inputs.positionRot = Units.rotationsToRadians(motorPosition.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
        inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
        inputs.currentAmps = new double[] {motorCurrent.getValueAsDouble()};
        
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        double motorVelocityRotPerSec =
                Units.radiansToRotations(velocityRadPerSec);;
        motor.setControl(
                switch (motorClosedLoopOutput) {
                    case Voltage -> velocityVoltageRequest.withVelocity(motorVelocityRotPerSec);
                    case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(motorVelocityRotPerSec);
                });
    }
    @Override
    public void setPosition(double position) {
        motor.setControl(
                switch (motorClosedLoopOutput) {
                    case Voltage -> voltageRequest.withOutput(position);
                    case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(position);
                });

    }
    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        this.configurePIDF(kP, kI, kD, 0);
    }
    @Override
    public void configurePIDF(double kP, double kI, double kD, double kF) {
        driveConfig.Slot0.kP = kP;
        driveConfig.Slot0.kI = kI;
        driveConfig.Slot0.kD = kD;
        driveConfig.Slot0.kS = kF;
        tryUntilOk(5, () -> motor.getConfigurator().apply(driveConfig, 0.25));

    }
    @Override
    public void resetPosition() {
        this.setOffset(0);
    }
    @Override
    public void setOffset(double offset) {
        motor.setPosition(offset);
    }
}
