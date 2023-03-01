package frc.robot.subsystems;





import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class Intake extends SubsystemBase {
   // private final CANSparkMax wristMotor;
    private final CANSparkMax intakeMotor; 
    private final RelativeEncoder outtakEncoder; 
    private final CANSparkMax wristMotor; 
    private double currentPosition; 

    public Intake(){
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless); 
        wristMotor = new CANSparkMax(Constants.Wrist.wristMotorId, MotorType.kBrushless);
       
        outtakEncoder = intakeMotor.getEncoder(); 
        
        outtakEncoder.setPositionConversionFactor(IntakeConstants.kDriveEncoderRot2Meter); 
        outtakEncoder.setVelocityConversionFactor(IntakeConstants.kDriveEncoderRPM2MeterPerSec); 
        
        resetEncoders(); 
        
    }
    public void setMotor(double speed){
        intakeMotor.set(speed); 
    }
    
    public double getPDMCurrent(){
        return intakeMotor.getOutputCurrent(); 
    }
    @Override 
    public void periodic(){
        //returns in amps
        // double intakeCurrent = pdm.getCurrent(Constants.IntakeConstants.pdpChannel); 
        double intakeCurrent = intakeMotor.getOutputCurrent();  
        SmartDashboard.putNumber("Intake Current", intakeCurrent); 
        SmartDashboard.putNumber("Wist Motor move", -wristMotor.getEncoder().getPosition());
    }  
    public void resetEncoders(){
        outtakEncoder.setPosition(0); 
        
    }
    public void resetEncoder(){
        wristMotor.getEncoder().setPosition(0);
       
    }
    public void move(double speed){
        wristMotor.set(speed*Constants.Wrist.maxMotorSpeed); 
    }
    
    
    
}