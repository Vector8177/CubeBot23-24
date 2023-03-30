package frc.robot.subsystems.intake;

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

    private final IntakeIO io; 
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged(); 
    /* Game Piece Currently In Robot */

    /**
     * Constructor for intake subsystem.
     */
    public Intake(IntakeIO io) {
        this.io = io; 
    
        intakeMotor = new CANSparkMax(Constants.Intake.motorId, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();

        

    }
    /*
     * public void setGamePiece(GamePiece piece) {
     * gamePiece = piece;
     * }
     * 
     * public GamePiece getGamePiece() {
     * return gamePiece;
     * }
     */

    public void setMotor(double speed) {
        intakeMotor.setVoltage(speed);
    }

    public double getPDMCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public double getVelocity() {
        return intakeEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        // returns in amps
        // double intakeCurrent = pdm.getCurrent(Constants.IntakeConstants.pdpChannel);
        double intakeCurrent = intakeMotor.getOutputCurrent();
        SmartDashboard.putNumber("Intake Current", intakeCurrent);
        SmartDashboard.putNumber("Intake Velocity", getVelocity());
        // SmartDashboard.putNumber("Gamepiece", getGamePiece().getDirection());

        io.updateInputs(inputs); 
        Logger.getInstance().processInputs("Input", inputs);
        

    }
    
    public void resetIntakeEncoder() {
        intakeEncoder.setPosition(0);

    }

}