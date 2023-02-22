package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase{

    private final CANSparkMax wristMotor; 

    public Wrist(){
        wristMotor = new CANSparkMax(WristConstants.wristMotorId, MotorType.kBrushless);
    }
}
