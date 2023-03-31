package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.Constants.Wrist.PIDFFmode;
import frc.robot.subsystems.wrist.WristIO.WristIOInputs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.AbsoluteEncoder;
import org.littletonrobotics.junction.Logger;


public class Wrist extends SubsystemBase {

    private double goalPosition;
    private final PIDController pidController;
    private ArmFeedforward feedForward;

    private final WristIO io; 
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged(); 

    public Wrist(WristIO io) {

        this.io = io; 
        pidController = new PIDController(Constants.Wrist.unweightedP, Constants.Wrist.unweightedI,
                Constants.Wrist.unweightedD);
        pidController.enableContinuousInput(0, Math.PI * 2);
        pidController.setTolerance(.25);

        
        feedForward = new ArmFeedforward(Constants.Wrist.unweightedS, Constants.Wrist.unweightedG,
                Constants.Wrist.unweightedV,
                Constants.Wrist.unweightedA);


        setPosition(Position.STANDBY.getWrist());

      
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Wrist Relative Encoder",
        // relativeEncoder.getPosition());
        SmartDashboard.putBoolean("Wrist at setpoint", atSetpoint());
        SmartDashboard.putNumber("Wrist Absolute Position", inputs.absoluteEncoderPosition);
        SmartDashboard.putNumber("Wrist Goal position", goalPosition);

        double pidMotorSpeed = pidController.calculate(inputs.absoluteEncoderPosition, goalPosition)
                + feedForward.calculate(goalPosition, 0);
        SmartDashboard.putNumber("Motor power wrist", pidMotorSpeed);
        setMotor(
                MathUtil.clamp(
                        (pidMotorSpeed), -Constants.Wrist.maxMotorVoltage,
                        Constants.Wrist.maxMotorVoltage));
                        Logger.getInstance().recordOutput("Goal position", getPosition()); 

    }

    public void setPIDFFMode(PIDFFmode mode) {
        pidController.setPID(mode.kP, mode.kI, mode.kD);
        feedForward = new ArmFeedforward(mode.kS, mode.kG, mode.kV, mode.kA);
        if(mode == PIDFFmode.WEIGHTED){
        SmartDashboard.putNumber("PIDFF", 1);
        } else{
            SmartDashboard.putNumber("PIDFF", 0);
        }
        
    }

   

    public double getEncoderPosition() {
        return inputs.absoluteEncoderPosition;
    }

    public void setMotor(double voltage) {
        io.setVoltage(voltage);
    }

    public Command setPositionCMD(double position) {
        return run(() -> setPosition(position)).until(() -> atSetpoint());
    }

    public void setPosition(double position) {
        goalPosition = position;
    }

    public double getPosition() {
        return goalPosition;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
    
    public void resetRelativeEncoder() {
        io.resetRelativeEncoder();

    }


}