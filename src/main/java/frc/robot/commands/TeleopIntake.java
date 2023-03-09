package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
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
        double maxSpeed = RobotContainer.gamePiece == GamePiece.CONE ? Constants.Intake.coneIntakeSpeed : Constants.Intake.cubeIntakeSpeed;
        s_Intake.setMotor(MathUtil.clamp(((moveVal.getAsDouble()) * maxSpeed + .5) * RobotContainer.gamePiece.getDirection()
        , -maxSpeed, maxSpeed));
    }
}
