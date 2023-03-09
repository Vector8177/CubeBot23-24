package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier leftBumper;
  private BooleanSupplier rightBumper;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(4.5);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(4.5);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.5);

  /**
   * The constructor initializes the class variables.
   * 
   * @param s_Swerve
   * @param translationSup
   * @param strafeSup
   * @param rotationSup
   * @param autoCenter
   * @param robotCentricSup
   */
  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier leftBumper,
      BooleanSupplier rightBumper) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.leftBumper = leftBumper;
    this.rightBumper = rightBumper;
  }

  /**
   * TODO
   */

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = translationLimiter
        .calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal = strafeLimiter
        .calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal = rotationLimiter
        .calculate(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(
          leftBumper.getAsBoolean() ? 
          Constants.Swerve.maxSpeedMaxLimit : 
          rightBumper.getAsBoolean() ? 
          Constants.Swerve.maxSpeedMinLimit : 
          Constants.Swerve.maxSpeed),
        rotationVal * (leftBumper.getAsBoolean() ? 
          Constants.Swerve.maxAngularVelocityMaxLimit : 
          rightBumper.getAsBoolean() ? 
          Constants.Swerve.maxAngularVelocityMinLimit : 
          Constants.Swerve.maxAngularVelocity),
        !robotCentricSup.getAsBoolean(),
        false);
  }
}
