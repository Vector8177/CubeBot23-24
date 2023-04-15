package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    /** Constructor for intake subsystem. */
    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void setMotor(double speed) {
        io.setVoltage(speed);
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);
    }

    public void setIntakeEncoder(double position) {
        io.setPosition(position);
    }
}
