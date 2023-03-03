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
    private final PIDController wristController;
    private final AbsoluteEncoder absoluteEncoder; 

    /**
     * Intake 
     */
    public Wrist(){
       
        wristMotor = new CANSparkMax(Constants.Wrist.wristMotorId, MotorType.kBrushless);
       
       relativeEncoder = wristMotor.getEncoder(); 
       absoluteEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle); 

        wristController = new PIDController(.2, 0, 0);
    }
    
   
    
    @Override 
    public void periodic(){
        SmartDashboard.putNumber("Wist Motor move", -wristMotor.getEncoder().getPosition());
    }  
    
    
    public void resetRelativeEncoder(){
        wristMotor.getEncoder().setPosition(0);
       
    }
    public void setWristMotor(double speed){
        wristMotor.set(speed*Constants.Wrist.maxMotorSpeed); 
    }
    
    
    
}