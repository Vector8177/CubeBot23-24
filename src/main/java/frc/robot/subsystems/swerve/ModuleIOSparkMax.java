package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Robot;

public class ModuleIOSparkMax implements ModuleIO {
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

    public ModuleIOSparkMax(SwerveModuleConstants moduleConstants) {
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

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.absoluteEncoder = angleEncoder.getAbsolutePosition();

        inputs.drivePosition = driveEncoder.getPosition();
        inputs.driveVelocityPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};
        inputs.driveTempCelcius = new double[] {driveMotor.getMotorTemperature()};

        inputs.turnAbsolutePosition = angleEncoder.getAbsolutePosition();
        inputs.turnPosition = integratedAngleEncoder.getPosition();
        inputs.turnVelocityPerSec = integratedAngleEncoder.getVelocity();
        inputs.turnAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {angleMotor.getOutputCurrent()};
        inputs.turnTempCelcius = new double[] {angleMotor.getMotorTemperature()};
    }

    @Override
    public void setMotorOutput(double percentOutput) {
        driveMotor.set(percentOutput);
    }

    public void setPosition(double absoultePosition) {
        integratedAngleEncoder.setPosition(absoultePosition);
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
        angleMotor.setInverted(SwerveConstants.angleInvert);
        angleMotor.setIdleMode(SwerveConstants.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(SwerveConstants.angleConversionFactor);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMinInput(-180.0);
        angleController.setPositionPIDWrappingMaxInput(180.0);
        angleController.setP(SwerveConstants.angleKP);
        angleController.setI(SwerveConstants.angleKI);
        angleController.setD(SwerveConstants.angleKD);
        angleController.setFF(SwerveConstants.angleKFF);
        angleMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    /** */
    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    /** */
    @Override
    public void resetToAbsolute() {
        double absolutePosition =
                angleEncoder.getAbsolutePosition() - moduleConstants.angleOffset.getDegrees();
        setPosition(absolutePosition);
    }

    /** */
    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
        driveMotor.setSmartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
        driveMotor.setInverted(SwerveConstants.driveInvert);
        driveMotor.setIdleMode(SwerveConstants.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(SwerveConstants.driveConversionPositionFactor);
        driveController.setP(SwerveConstants.angleKP);
        driveController.setI(SwerveConstants.angleKI);
        driveController.setD(SwerveConstants.angleKD);
        driveController.setFF(SwerveConstants.angleKFF);
        driveMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    @Override
    public void setVelocity(SwerveModuleState desiredState, double ffVoltage) {
        driveController.setReference(
                desiredState.speedMetersPerSecond, ControlType.kVelocity, 0, ffVoltage);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    }
}
