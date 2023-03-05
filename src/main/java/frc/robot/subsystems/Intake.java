package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The intake subsysatem will be used to set up the motors and encoders for the
 * intake.
 */
public class Intake extends SubsystemBase {

    private final CANSparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    /**
     * Constructor for intake subsystem.
     */
    public Intake() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorId, MotorType.kBrushless);

        intakeEncoder = intakeMotor.getEncoder();

        intakeEncoder.setPositionConversionFactor(Constants.Intake.kDriveEncoderRot2Meter);

        intakeEncoder.setVelocityConversionFactor(Constants.Intake.kDriveEncoderRPM2MeterPerSec);

    }

    public void setMotor(double speed) {
        intakeMotor.set(speed);
    }

    public double getPDMCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        // returns in amps
        // double intakeCurrent = pdm.getCurrent(Constants.IntakeConstants.pdpChannel);
        double intakeCurrent = intakeMotor.getOutputCurrent();
        SmartDashboard.putNumber("Intake Current", intakeCurrent);

    }

    public void resetIntakeEncoder() {
        intakeEncoder.setPosition(0);

    }

}