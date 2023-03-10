package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalancing extends CommandBase {
    private Swerve s_Swerve;

    /**
     * 
     * @param s_Swerve TODO
     */

    public AutoBalancing(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {

        double translationVal = s_Swerve.getPitch().getDegrees() > 0 ? -.25 : .25;
        s_Swerve.drive(
                new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed), 0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(
                new Translation2d(0, .1).times(Constants.Swerve.maxSpeed), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Swerve.getPitch().getDegrees()) < 5;
    }
}