package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.AbsoluteEncoder;

/**
 * The wrist subsystem will be used to set up the motors and encoders for the
 * wrist.
 */
public class Wrist extends SubsystemBase {
    private final CANSparkMax wristMotor;

    /**
     * Declares the relative and absolute encoders for the wrist. The absolute
     * encoder
     * is the through bore encoder and the relative encoder is the encoder that is
     * built into the
     * spark max motor.
     */
    private final RelativeEncoder relativeEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    private double currentPosition;
    private final PIDController pidController;
    private final ArmFeedforward feedForward;

    /**
     * Intake
     */
    public Wrist() {

        wristMotor = new CANSparkMax(Constants.Wrist.wristMotorId, MotorType.kBrushless);
        wristMotor.setInverted(true);
        wristMotor.setIdleMode(IdleMode.kBrake);

        absoluteEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setInverted(true);

        pidController = new PIDController(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD);
        pidController.enableContinuousInput(0, Math.PI * 2);
        pidController.setTolerance(.25);

        relativeEncoder = wristMotor.getEncoder();

        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setZeroOffset(Constants.Wrist.absoluteEncoderOffset);
        relativeEncoder.setPositionConversionFactor(Constants.Wrist.motorGearRatio * 2 * Math.PI);

        feedForward = new ArmFeedforward(Constants.Wrist.kS, Constants.Wrist.kG, Constants.Wrist.kV,
                Constants.Wrist.kA);

        relativeEncoder.setPosition(absoluteEncoder.getPosition());

        setPosition(Position.STANDBY.getWrist());

        wristMotor.setSmartCurrentLimit(Constants.Wrist.currentLimit); 
    }

    public void resetRelativeEncoder() {
        wristMotor.getEncoder().setPosition(0);

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Wrist Relative Encoder",
        // relativeEncoder.getPosition());
        SmartDashboard.putBoolean("Wrist at setpoint", atSetpoint());
        SmartDashboard.putNumber("Wrist Absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Goal position", currentPosition);
        
        double pidMotorSpeed = pidController.calculate(absoluteEncoder.getPosition(), currentPosition) + feedForward.calculate(currentPosition, 0) ;
        SmartDashboard.putNumber("Motor power wrist", pidMotorSpeed);
        setMotor(
                MathUtil.clamp(
                        (pidMotorSpeed)
                        ,-Constants.Wrist.maxMotorVoltage,
                        Constants.Wrist.maxMotorVoltage)
                        );
                        
        
    }

    public void resetEncoder() {
        wristMotor.getEncoder().setPosition(0);

    }

    public double getEncoderPosition(){
        return absoluteEncoder.getPosition();
    }

    public void setMotor(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    public Command setPose(double position) {
        return run(() -> setPosition(position)).until(() -> atSetpoint());
    }

    public void setPosition(double position) {
        currentPosition = position;
    }

    public double getPosition() {
        return currentPosition;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

}