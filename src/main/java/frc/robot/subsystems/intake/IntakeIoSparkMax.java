package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIoSparkMax implements IntakeIo{
    private final CANSparkMax intakeMotor; 
    private final RelativeEncoder intakeEncoder; 
    public IntakeIoSparkMax(){
        intakeMotor = new CANSparkMax(Constants.Intake.motorId, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
    }
}
