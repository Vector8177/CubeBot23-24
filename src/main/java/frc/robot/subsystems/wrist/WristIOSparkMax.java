package frc.robot.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import frc.robot.Constants;
import frc.robot.subsystems.wrist.WristIO.WristIOInputs;

public class WristIOSparkMax {
    private final RelativeEncoder relativeEncoder;
    private final CANSparkMax wristMotor;
    private final AbsoluteEncoder absoluteEncoder; 

    public WristIOSparkMax(){
        wristMotor = new CANSparkMax(Constants.Wrist.wristMotorId, MotorType.kBrushless);
        wristMotor.setInverted(true);
        wristMotor.setIdleMode(IdleMode.kBrake);

      absoluteEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setInverted(true);

        
        relativeEncoder = wristMotor.getEncoder();

       absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
       absoluteEncoder.setZeroOffset(Constants.Wrist.absoluteEncoderOffset);
        relativeEncoder.setPositionConversionFactor(Constants.Wrist.motorGearRatio * 2 * Math.PI);

      

 relativeEncoder.setPosition(absoluteEncoder.getPosition());


        wristMotor.setSmartCurrentLimit(Constants.Wrist.currentLimit);

    }
    public void updateInputs(WristIOInputs inputs){
        inputs.velocity = relativeEncoder.getVelocity(); 
        inputs.current = wristMotor.getOutputCurrent(); 

        inputs.absoluteEncoderPosition = absoluteEncoder.getPosition(); 
        

    }
    public void resetEncoder(){
        relativeEncoder.setPosition(0); 
    }
    public void setVoltage(double voltage){
        wristMotor.setVoltage(voltage); 
    }
}
