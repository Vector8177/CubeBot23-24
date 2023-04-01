package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class ModuleIOSparkMax implements ModuleIO{
    public int moduleNumber;
    public SwerveModuleConstants moduleConstants;
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    /* Initialize PID drive and angle */
    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController angleController;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder angleEncoder;

    public double position = 0.0; 
    
   

    public ModuleIOSparkMax(int moduleNumber, SwerveModuleConstants moduleConstants){
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        this.moduleConstants = moduleConstants;

         /* Angle Motor Config */
         angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
         integratedAngleEncoder = angleMotor.getEncoder();
         angleController = angleMotor.getPIDController();
         configAngleMotor();
 
         /* Drive Motor Config */
         driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
         driveEncoder = driveMotor.getEncoder();
         driveController = driveMotor.getPIDController();
         configDriveMotor();
    }

    public void updateInputs(ModuleIOInputs inputs){
        inputs.absoluteEncoder = Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
        inputs.position = new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(integratedAngleEncoder.getPosition()));
        inputs.state = new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(integratedAngleEncoder.getPosition()));
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};
        inputs.driveTempCelcius = new double[] {driveMotor.getMotorTemperature()};
        inputs.turnAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {angleMotor.getOutputCurrent()};
        inputs.turnTempCelcius = new double[] {angleMotor.getMotorTemperature()};
    }
    public void setMotorOutput(double percentOutput){
        driveMotor.set(percentOutput); 
    }
    public void setPosition(double absoultePosition){
        integratedAngleEncoder.setPosition(absoultePosition); 
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.Swerve.angleInvert);
        angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMinInput(-180.0);
        angleController.setPositionPIDWrappingMaxInput(180.0);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKFF);
        angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    /**
     * 
     */
    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    /**
     * 
     */
    public void resetToAbsolute() {
        double absolutePosition = angleEncoder.getAbsolutePosition() - moduleConstants.angleOffset.getDegrees();
        setPosition(absolutePosition);
    }

    /**
     * 
     */
    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.Swerve.driveInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveController.setP(Constants.Swerve.angleKP);
        driveController.setI(Constants.Swerve.angleKI);
        driveController.setD(Constants.Swerve.angleKD);
        driveController.setFF(Constants.Swerve.angleKFF);
        driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    public void setVelocity(SwerveModuleState desiredState, double ffVoltage){
        driveController.setReference(
                        desiredState.speedMetersPerSecond,
                        ControlType.kVelocity,
                        0,
                        ffVoltage);
    }

    public void setAngle(Rotation2d angle){
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    }
}
