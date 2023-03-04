package frc.robot.subsystems;





import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import com.revrobotics.AbsoluteEncoder;

/**
 * The wrist subsystem will be used to set up the motors and encoders for the wrist.  
 */
public class Wrist extends SubsystemBase {
    private final CANSparkMax wristMotor; 
    
    /**Declares the relative and absolute encoders for the wrist. The absolute encoder
        is the through bore encoder and the relative encoder is the encoder that is built into the 
        spark max motor. 
    */
    private final RelativeEncoder relativeEncoder; 
    private final AbsoluteEncoder absoluteEncoder; 

    private double currentPosition; 
    private final PIDController pidController;
    private final ArmFeedforward feedForward;
    

    /**
     * Intake 
     */
    public Wrist(){
       
        wristMotor = new CANSparkMax(Constants.Wrist.wristMotorId, MotorType.kBrushless);
        wristMotor.setInverted(true);
       
       absoluteEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle); 
       absoluteEncoder.setInverted(true);

        pidController = new PIDController(5, .3, .1);
        pidController.enableContinuousInput(0, Math.PI*2);

        relativeEncoder = wristMotor.getEncoder();
       
        absoluteEncoder.setPositionConversionFactor(2*Math.PI);
        absoluteEncoder.setZeroOffset(5.412927);
        relativeEncoder.setPositionConversionFactor(IntakeConstants.kWristMotorGearRatio * 2*Math.PI); 
        
        feedForward = new ArmFeedforward(0.6,1.36,.62,0); 

        relativeEncoder.setPosition(absoluteEncoder.getPosition());

        currentPosition = 0;
    }
    
    
    
    public void resetRelativeEncoder(){
        wristMotor.getEncoder().setPosition(0);
       
    }
    
    
    @Override
    public void periodic(){
    SmartDashboard.putNumber("Wrist Relative Encoder", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Absolute Encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Goal wrist position", currentPosition);

    double pidMotorSpeed = pidController.calculate(absoluteEncoder.getPosition(), currentPosition);
    
    SmartDashboard.putNumber("motor power wrist", pidMotorSpeed);
    setWristMotor(
        MathUtil.clamp(
            (pidMotorSpeed),
            -3,
            3));
}  

public void resetWristEncoder(){
    wristMotor.getEncoder().setPosition(0);
   
}
public void setWristMotor(double voltage) {
    wristMotor.setVoltage(voltage*Constants.Wrist.maxMotorSpeed); 
}

public void setWristPosition(double position){
    currentPosition = position;
}

public double getWristPosition(){
    return currentPosition;
}

    
}