package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
    private static final double GEAR_RATIO = 1.5;

    private final TalonFX motor = new TalonFX(15);

    private final StatusSignal<Angle> motorPosition = motor.getPosition();
    private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
    private final StatusSignal<Voltage> motorAppliedVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();

    public IntakeIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
        motor.optimizeBusUtilization();
    }

    /** @param inputs */
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
        inputs.positionRad = Units.rotationsToRadians(motorPosition.getValueAsDouble()) / GEAR_RATIO;
        inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble()) / GEAR_RATIO;
        inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
        inputs.currentAmps = new double[] {motorCurrent.getValueAsDouble()};
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        motor.setControl(new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        var config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        motor.getConfigurator().apply(config);
    }
}
