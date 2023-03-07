package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Position;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The intake subsysatem will be used to set up the motors and encoders for the
 * intake.
 */
public class Intake extends SubsystemBase {

    private final CANSparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    /* Game Piece Currently In Robot */
    private static GamePiece gamePiece = GamePiece.CUBE;

    /**
     * Constructor for intake subsystem.
     */
    public Intake() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorId, MotorType.kBrushless);

        intakeEncoder = intakeMotor.getEncoder();

        
    }

    public static void setGamePiece(GamePiece piece) {
        gamePiece = piece;
    }

    public static GamePiece getGamePiece() {
        return gamePiece;
    }

    public void setMotor(double speed) {
        intakeMotor.setVoltage(speed);
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
        SmartDashboard.putNumber("Gamepiece", getGamePiece().getDirection());

    }

    public void resetIntakeEncoder() {
        intakeEncoder.setPosition(0);

    }

}