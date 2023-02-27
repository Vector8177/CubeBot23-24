package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import com.revrobotics.AbsoluteEncoder;

public class Intake extends SubsystemBase {
    private final CANSparkMax wristMotor;
    private final CANSparkMax intakeMotor; 
    private  AbsoluteEncoder outtakEncoder; 
    private final SparkMaxPIDController wristPidController; 

    public Intake(){
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless); 
        wristMotor = new CANSparkMax(Constants.IntakeConstants.wristMotorId, MotorType.kBrushless);
        // pdm = new PowerDistribution(1, PowerDistribution.ModuleType.kRev); 
        outtakEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle); 
        
        outtakEncoder.setPositionConversionFactor(IntakeConstants.kDriveEncoderRot2Meter); 
        outtakEncoder.setVelocityConversionFactor(IntakeConstants.kDriveEncoderRPM2MeterPerSec); 
        
        wristPidController = wristMotor.getPIDController(); 
        wristPidController.setP(Constants.IntakeConstants.kP); 
        wristPidController.setI(Constants.IntakeConstants.kI); 
        wristPidController.setD(Constants.IntakeConstants.kD); 
        wristPidController.setIZone(Constants.IntakeConstants.kIz); 
        wristPidController.setFF(Constants.IntakeConstants.kFF); 
        wristPidController.setOutputRange(Constants.IntakeConstants.kMinOutput, Constants.IntakeConstants.kMaxOutput);
        
        SmartDashboard.putNumber("P Gain", Constants.IntakeConstants.kP);
        SmartDashboard.putNumber("I Gain", Constants.IntakeConstants.kI);
        SmartDashboard.putNumber("D Gain", Constants.IntakeConstants.kD);
        SmartDashboard.putNumber("I Zone", Constants.IntakeConstants.kIz);
        SmartDashboard.putNumber("Feed Forward", Constants.IntakeConstants.kFF);
        SmartDashboard.putNumber("Max Output", Constants.IntakeConstants.kMaxOutput);
        SmartDashboard.putNumber("Min Output", Constants.IntakeConstants.kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
       
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


        //This is to allow for easy pid adjusting
         // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != Constants.IntakeConstants.kP)) { 
            wristPidController.setP(p); Constants.IntakeConstants.kP = p; }
        if((i != Constants.IntakeConstants.kI)) {
             wristPidController.setI(i); Constants.IntakeConstants.kI = i; }
        if((d != Constants.IntakeConstants.kD)) {
            wristPidController.setD(d); Constants.IntakeConstants.kD = d; }
        if((iz != Constants.IntakeConstants.kIz)) { 
            wristPidController.setIZone(iz); Constants.IntakeConstants.kIz = iz; }
        if((ff != Constants.IntakeConstants.kFF)) {
             wristPidController.setFF(ff); Constants.IntakeConstants.kFF = ff; }
        if((max != Constants.IntakeConstants.kMaxOutput) || (min != Constants.IntakeConstants.kMinOutput)) { 
            wristPidController.setOutputRange(min, max); 
        Constants.IntakeConstants.kMinOutput = min; Constants.IntakeConstants.kMaxOutput = max; 
    }


    }  
    public void resetEncoders(){
        //outtakEncoder.setPosition(0); 
        
    }
    
}