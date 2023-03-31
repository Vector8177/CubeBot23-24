package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOSparkMax implements IntakeIO{
    private final CANSparkMax intakeMotor; 
    private final RelativeEncoder intakeEncoder; 

    public IntakeIOSparkMax(){
        intakeMotor = new CANSparkMax(Constants.Intake.motorId, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
    }
    public default void updateInputs(IntakeIOInputs inputs){
        inputs.velocity = intakeEncoder.getVelocity();
        inputs.current = intakeMotor.getOutputCurrent();
    }

    public default void setVoltage(double speed){
        intakeMotor.setVoltage(speed);
    }

    public default void setPosition(double position){
        intakeEncoder.setPosition(position);
    }
}
