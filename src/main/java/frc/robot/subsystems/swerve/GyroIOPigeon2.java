package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(Constants.Swerve.pigeonID);

    pigeon.configFactoryDefault();
    pigeon.zeroGyroBiasNow();
    pigeon.setYaw(0.0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    pigeon.getYawPitchRoll(yprDegrees);
    pigeon.getRawGyro(xyzDps);
    inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
    inputs.rollPosition = yprDegrees[1];
    inputs.pitchPosition = -yprDegrees[2];
    inputs.yawPosition = yprDegrees[0];
    inputs.rollVelocityPerSec = xyzDps[1];
    inputs.pitchVelocityPerSec = -xyzDps[0];
    inputs.yawVelocityPerSec = xyzDps[2];
  }

  @Override
  public void setYaw(Rotation2d angle) {
    pigeon.setYaw(angle.getDegrees());
  }
}