package frc.robot.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class WristIOSparkMax implements WristIO {
    private final RelativeEncoder relativeEncoder;
    private final CANSparkMax wristMotor;
    private final AbsoluteEncoder absoluteEncoder;

    public WristIOSparkMax() {
        wristMotor = new CANSparkMax(WristConstants.wristMotorId, MotorType.kBrushless);
        wristMotor.setInverted(true);
        wristMotor.setIdleMode(IdleMode.kBrake);

        absoluteEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setInverted(true);

        relativeEncoder = wristMotor.getEncoder();

        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setZeroOffset(WristConstants.absoluteEncoderOffset);
        relativeEncoder.setPositionConversionFactor(WristConstants.motorGearRatio * 2 * Math.PI);

        relativeEncoder.setPosition(absoluteEncoder.getPosition());

        wristMotor.setSmartCurrentLimit(WristConstants.currentLimit);

    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.velocity = relativeEncoder.getVelocity();
        inputs.absoluteEncoderPosition = absoluteEncoder.getPosition();

        inputs.appliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
        inputs.currentAmps = new double[] { wristMotor.getOutputCurrent() };
        inputs.tempCelcius = new double[] { wristMotor.getMotorTemperature() };

    }

    @Override
    public void resetRelativeEncoder() {
        relativeEncoder.setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }
}
