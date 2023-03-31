package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final CANSparkMax elevatorMotorLeft; // making the left the lead motor
    private final CANSparkMax elevatorMotorRight;

    public ElevatorIOSparkMax()
    {
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
        inputs.currentPosition = (elevatorMotorLeft.getEncoder().getPosition() + elevatorMotorRight.getEncoder().getPosition()) / 2;

    }
}
