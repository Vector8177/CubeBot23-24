package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ScoreGamePiece;

public class Elevator extends SubsystemBase {
    private final CANSparkMax elevatorMotorLeft;
    private final CANSparkMax elevatorMotorRight;

    private final CANCoder elevatorLeftController;
    private final CANCoder elevatorRightController;

    private PIDController pidController;

    private double currentPosition;



    public Elevator() {
        elevatorMotorLeft = new CANSparkMax(Constants.Elevator.motorLeftId, MotorType.kBrushless);
        elevatorLeftController = new CANCoder(Constants.Elevator.canConderLeftId);

        elevatorMotorRight = new CANSparkMax(Constants.Elevator.motorRightId, MotorType.kBrushless);
        elevatorMotorRight.setInverted(true);
        elevatorRightController = new CANCoder(Constants.Elevator.canConderRightId);

        pidController = new PIDController(Constants.Elevator.elevatorKP, Constants.Elevator.elevatorKI, Constants.Elevator.elevatorKD);
        pidController.setSetpoint(0);
        pidController.setTolerance(.1);
    }

    public void resetEncoder(){
        elevatorLeftController.setPosition(0);
        elevatorRightController.setPosition(0);
    }

    public void raise(double distance){
        updatePosition();
        distance -= currentPosition;
        this.move(MathUtil.clamp(pidController.calculate(distance), -Constants.Elevator.maxMotorSpeed, Constants.Elevator.maxMotorSpeed));
    }
    public void move(double speed){
        elevatorMotorLeft.set(speed/Constants.Elevator.maxMotorSpeed);
        elevatorMotorRight.set(speed/Constants.Elevator.maxMotorSpeed);
    }
    public boolean reachedSetpoint(double distance){
        return pidController.getPositionTolerance() <= Math.abs(currentPosition - distance);
    }

    private void updatePosition(){
        currentPosition += (elevatorLeftController.getPosition() + elevatorRightController.getPosition())/2;
    }

    public void score(int level, boolean cone){
        new ScoreGamePiece(this, level, cone);
    }
}