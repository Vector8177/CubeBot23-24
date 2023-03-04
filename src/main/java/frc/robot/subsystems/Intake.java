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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import com.revrobotics.AbsoluteEncoder;

/**
 * The intake subsysatem will be used to set up the motors and encoders for the intake. 
 */
public class Intake extends SubsystemBase {
  
    private final CANSparkMax intakeMotor; 
    private final RelativeEncoder intakeEncoder; 

   


    /**
     * Constructor for intake subsystem. 
     */
    public Intake(){
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless); 
       
        intakeEncoder = intakeMotor.getEncoder(); 
      

      
        intakeEncoder.setPositionConversionFactor(IntakeConstants.kDriveEncoderRot2Meter); 

        intakeEncoder.setVelocityConversionFactor(IntakeConstants.kDriveEncoderRPM2MeterPerSec); 
        
        
    }
    
    public void setMotor(double speed){
        intakeMotor.set(speed); 
    }

    public double getPDMCurrent() {
        return intakeMotor.getOutputCurrent();
    }
    @Override 
    public void periodic(){
        //returns in amps
        // double intakeCurrent = pdm.getCurrent(Constants.IntakeConstants.pdpChannel); 
        double intakeCurrent = intakeMotor.getOutputCurrent();  
        SmartDashboard.putNumber("Intake Current", intakeCurrent); 
        

    }  
    public void resetIntakeEncoder(){
        intakeEncoder.setPosition(0); 
        
    }
   
    
    
}