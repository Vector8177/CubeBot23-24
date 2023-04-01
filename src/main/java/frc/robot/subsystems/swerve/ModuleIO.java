package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public Rotation2d absoluteEncoder = new Rotation2d();
        public SwerveModulePosition position = new SwerveModulePosition();
        public SwerveModuleState state = new SwerveModuleState();

        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelcius = new double[] {};
    
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public double[] turnTempCelcius = new double[] {};
      }
    
      /** Updates the set of loggable inputs. */
      public default void updateInputs(ModuleIOInputs inputs) {}

      public default void resetToAbsolute(){}

      public default void setMotorOutput(double percentOutput){}

      public default void setVelocity(SwerveModuleState desiredState, double ffVoltage){}

      public default void setAngle(Rotation2d angle){}
    }

