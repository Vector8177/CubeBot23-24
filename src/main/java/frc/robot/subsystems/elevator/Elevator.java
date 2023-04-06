package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private PIDController pidController;
    private double targetPosition;

    public Elevator(ElevatorIO io) {
        this.io = io;
        // initialize pidContoller
        pidController = new PIDController(Constants.Elevator.elevatorKP, Constants.Elevator.elevatorKI,
                Constants.Elevator.elevatorKD);
        pidController.setSetpoint(0);
        pidController.setTolerance(.25);

        setPosition(Position.STANDBY.getElev());
    }

    public void setPosition(double targetPos) {
        if (targetPos > 35) {
            targetPos = 35;
        } else if (targetPos < 0.1) {
            targetPos = 0.1;
        }
        Logger.getInstance().recordOutput("ElevatorTargetPosition", targetPos);
        this.targetPosition = targetPos;
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public Command moveElevator(double targetPosition) {
        return run(() -> setPosition(targetPosition)).until(() -> atSetpoint());
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void move(double voltage) {
        io.move(voltage);
    }

    public boolean reachedSetpoint(double distance) {
        return pidController.getPositionTolerance() >= Math.abs(targetPosition - distance);
    }

    public double getEncoderPosition() {
        return inputs.currentPosition;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);

        SmartDashboard.putBoolean("Elevator at setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Position", getEncoderPosition());
        SmartDashboard.putNumber("Elevator Goal Position", inputs.currentPosition);

        move(
                MathUtil.clamp(
                        pidController.calculate(getEncoderPosition(), targetPosition),
                        -Constants.Elevator.maxMotorVoltage,
                        Constants.Elevator.maxMotorVoltage));
    }
}