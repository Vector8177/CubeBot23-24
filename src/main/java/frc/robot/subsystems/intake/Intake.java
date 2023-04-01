package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The intake subsysatem will be used to set up the motors and encoders for the
 * intake.
 */
public class Intake extends SubsystemBase {

   
    private final IntakeIO io; 
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged(); 
    /* Game Piece Currently In Robot */

    /**
     * Constructor for intake subsystem.
     */
    public Intake(IntakeIO io) {
        this.io = io; 
        

        

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
        io.setVoltage(speed);
    }

    public double getPDMCurrent() {
        return inputs.current;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    @Override
    public void periodic() {
        // returns in amps
        // double intakeCurrent = pdm.getCurrent(Constants.IntakeConstants.pdpChannel);
        double intakeCurrent = getPDMCurrent();
        SmartDashboard.putNumber("Intake Current", intakeCurrent);
        SmartDashboard.putNumber("Intake Velocity", getVelocity());
        // SmartDashboard.putNumber("Gamepiece", getGamePiece().getDirection());

        io.updateInputs(inputs); 
        Logger.getInstance().processInputs("Input", inputs);
        

    }
    
    public void resetIntakeEncoder() {
        io.setPosition(0);

    }

}