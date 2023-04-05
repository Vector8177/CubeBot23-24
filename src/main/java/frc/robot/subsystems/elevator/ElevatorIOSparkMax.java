package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final CANSparkMax elevatorMotorLeft; // making the left the lead motor
    private final CANSparkMax elevatorMotorRight;

    public ElevatorIOSparkMax() {
        elevatorMotorLeft = new CANSparkMax(Constants.Elevator.motorLeftId, MotorType.kBrushless);

        elevatorMotorRight = new CANSparkMax(Constants.Elevator.motorRightId, MotorType.kBrushless);

        elevatorMotorLeft.restoreFactoryDefaults();
        elevatorMotorRight.restoreFactoryDefaults();

        elevatorMotorRight.follow(elevatorMotorLeft, true);

        elevatorMotorLeft.setSmartCurrentLimit(Constants.Elevator.currentLimit);
        elevatorMotorRight.setSmartCurrentLimit(Constants.Elevator.currentLimit);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentPosition = (elevatorMotorLeft.getEncoder().getPosition()
                + elevatorMotorRight.getEncoder().getPosition()) / 2;

        inputs.leftAppliedVolts = elevatorMotorLeft.getAppliedOutput() * elevatorMotorLeft.getBusVoltage();
        inputs.leftCurrentAmps = elevatorMotorLeft.getOutputCurrent();
        inputs.leftTempCelcius = elevatorMotorLeft.getMotorTemperature();

        inputs.rightAppliedVolts = elevatorMotorRight.getAppliedOutput() * elevatorMotorRight.getBusVoltage();
        inputs.rightCurrentAmps = new double[] { elevatorMotorRight.getOutputCurrent() };
        inputs.rightTempCelcius = new double[] { elevatorMotorRight.getMotorTemperature() };
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
