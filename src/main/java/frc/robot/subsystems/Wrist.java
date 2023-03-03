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
 * The intake subsysatem will be used to set up the motors and encoders for the intake. 
 */
public class Wrist extends SubsystemBase {
    private final CANSparkMax wristMotor; 
    
    private final RelativeEncoder relativeEncoder; 
    private double currentPosition; 
    private final PIDController pidController;
    private final ArmFeedforward feedForward;
    private final AbsoluteEncoder absoluteEncoder; 

    /**
     * Intake 
     */
    public Wrist(){
       
        wristMotor = new CANSparkMax(Constants.Wrist.wristMotorId, MotorType.kBrushless);
       
       absoluteEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle); 

        pidController = new PIDController(.2, 0, 0);

        relativeEncoder = wristMotor.getEncoder();
       
        absoluteEncoder.setPositionConversionFactor(2*Math.PI);
        relativeEncoder.setPositionConversionFactor(IntakeConstants.kWristMotorGearRatio); 
        
        feedForward = new ArmFeedforward(0,0,0,0); 

        relativeEncoder.setPosition(absoluteEncoder.getPosition());

        currentPosition = 1;
    }
    
    
    
    public void resetRelativeEncoder(){
        wristMotor.getEncoder().setPosition(0);
       
    }
    
    //
    @Override
    public void periodic(){
    SmartDashboard.putNumber("Wrist Relative Encoder", -relativeEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Absolute Encoder", -absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Goal wrist position", currentPosition);

    double pidMotorSpeed = -pidController.calculate(-relativeEncoder.getPosition(), currentPosition);
    
    SmartDashboard.putNumber("motor power wrist", pidMotorSpeed);
    setWristMotor(
        MathUtil.clamp(
            pidMotorSpeed,
            -1,
            1));
}  

public void resetWristEncoder(){
    wristMotor.getEncoder().setPosition(0);
   
}
public void setWristMotor(double speed) {
    wristMotor.set(speed*Constants.Wrist.maxMotorSpeed); 
}

public void setWristPosition(double position){
    currentPosition = position;
}

public double getWristPosition(){
    return currentPosition;
}

    
}