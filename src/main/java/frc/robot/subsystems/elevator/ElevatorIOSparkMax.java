package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final CANSparkMax elevatorMotorLeft; // making the left the lead motor
    private final CANSparkMax elevatorMotorRight;

    public ElevatorIOSparkMax() {
        elevatorMotorLeft = new CANSparkMax(ElevatorConstants.motorLeftId, MotorType.kBrushless);

        elevatorMotorRight = new CANSparkMax(ElevatorConstants.motorRightId, MotorType.kBrushless);

        elevatorMotorLeft.restoreFactoryDefaults();
        elevatorMotorRight.restoreFactoryDefaults();

        elevatorMotorRight.follow(elevatorMotorLeft, true);

        elevatorMotorLeft.setSmartCurrentLimit(ElevatorConstants.currentLimit);
        elevatorMotorRight.setSmartCurrentLimit(ElevatorConstants.currentLimit);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentPosition =
                (elevatorMotorLeft.getEncoder().getPosition()
                                + elevatorMotorRight.getEncoder().getPosition())
                        / 2;

        inputs.leftAppliedVolts =
                elevatorMotorLeft.getAppliedOutput() * elevatorMotorLeft.getBusVoltage();
        inputs.leftCurrentAmps = new double[] {elevatorMotorLeft.getOutputCurrent()};
        inputs.leftTempCelcius = new double[] {elevatorMotorLeft.getMotorTemperature()};

        inputs.rightAppliedVolts =
                elevatorMotorRight.getAppliedOutput() * elevatorMotorRight.getBusVoltage();
        inputs.rightCurrentAmps = new double[] {elevatorMotorRight.getOutputCurrent()};
        inputs.rightTempCelcius = new double[] {elevatorMotorRight.getMotorTemperature()};
    }

    @Override
    public void resetEncoder() {
        elevatorMotorLeft.getEncoder().setPosition(0);
        elevatorMotorRight.getEncoder().setPosition(0);
    }

    @Override
    public void move(double voltage) {
        elevatorMotorLeft.setVoltage(voltage);
    }
}
