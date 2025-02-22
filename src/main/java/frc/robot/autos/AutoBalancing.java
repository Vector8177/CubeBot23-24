package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

public class AutoBalancing extends CommandBase {
    private Swerve s_Swerve;
    private boolean sideways;

    /**
     * @param s_Swerve TODO
     */
    public AutoBalancing(Swerve s_Swerve, boolean sideways) {
        this.s_Swerve = s_Swerve;
        this.sideways = sideways;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        if (!sideways) {
            double translationVal = s_Swerve.getPitch().getDegrees() > 0 ? -.45 : .45;
            s_Swerve.drive(new Translation2d(translationVal, 0), 0, false, true);
        } else {
            double translationVal = s_Swerve.getRoll().getDegrees() > 0 ? .435 : -.435;
            s_Swerve.drive(new Translation2d(0, translationVal), 0, false, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, .3), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        if (!sideways) {
            return Math.abs(s_Swerve.getPitch().getDegrees()) < 7;
        } else {
            return Math.abs(s_Swerve.getRoll().getDegrees()) < 7;
        }
    }
}
