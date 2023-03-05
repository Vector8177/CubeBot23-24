package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends CommandBase {
    private Intake s_Intake;
    private DoubleSupplier moveVal;

    public TeleopIntake(Intake s_Intake, DoubleSupplier moveVal) {
        this.s_Intake = s_Intake;
        this.moveVal = moveVal;

        addRequirements(s_Intake);
    }

    @Override
    public void execute() {
            s_Intake.setMotor(MathUtil.clamp(moveVal.getAsDouble(), 0, Constants.IntakeConstants.intakeSpeed) * RobotContainer.gamePiece.getDirection());
    }
}
