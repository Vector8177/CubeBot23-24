package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class TeleopTestIntake extends CommandBase {
    private Intake s_Intake;
    private DoubleSupplier wristAxis;

    public TeleopTestIntake(
        Intake s_Intake,
        DoubleSupplier wristAxis
        ) {
      this.s_Intake = s_Intake;
      addRequirements(s_Intake);

      this.wristAxis = wristAxis;
    }

    @Override
  public void execute() {
    s_Intake.setWristMotor(wristAxis.getAsDouble() * Constants.IntakeConstants.wristSpeed);
  }


}
