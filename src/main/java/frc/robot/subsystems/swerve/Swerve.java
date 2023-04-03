package frc.robot.subsystems.swerve;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;

public class Swerve extends SubsystemBase {

    private SwerveDrivePoseEstimator swervePoseEstimator;
    private Module[] mSwerveMods;

    private GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private ChassisSpeeds chassisSpeeds;

    private final Vision s_Vision;

    private Field2d field;

    public Swerve(GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Vision s_Vision) {

        this.gyroIO = gyroIO;
        setYaw(Rotation2d.fromDegrees(0));

        mSwerveMods = new Module[] {
                new Module(flModuleIO, 0),
                new Module(frModuleIO, 1),
                new Module(blModuleIO, 2),
                new Module(brModuleIO, 3)
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(),
                new Pose2d());

        this.s_Vision = s_Vision;

        field = new Field2d();
        chassisSpeeds = new ChassisSpeeds();
        SmartDashboard.putData(field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.fastSpeedLimit);

        for (Module mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.index], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.fastSpeedLimit);

        for (Module mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.index], false);
        }
    }

    public void setModuleRotation(Rotation2d rotation) {
        for (Module mod : mSwerveMods) {
            mod.setDesiredState(new SwerveModuleState(0, rotation), false);
        }
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public Field2d getField() {
        return field;
    }

    public void resetOdometry(Pose2d pose) {
        swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);

    }

    public void resetToAbsolute() {
        for (Module mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (Module mod : mSwerveMods) {
            states[mod.index] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (Module mod : mSwerveMods) {
            positions[mod.index] = mod.getPosition();
        }
        return positions;
    }

    public void setYaw(Rotation2d angle) {
        gyroIO.setYaw(angle); // Angle in Degrees
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyroInputs.pitchPosition);
    }

    public PathPoint getPoint() {
        return PathPoint.fromCurrentHolonomicState(getPose(), chassisSpeeds);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(360 - gyroInputs.yawPosition)
                : Rotation2d.fromDegrees(gyroInputs.yawPosition);
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyroInputs.rollPosition);
    }

    public Vision getCamera() {
        return s_Vision;
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        for (Module mod : mSwerveMods) {
            mod.periodic();
        }

        swervePoseEstimator.update(getYaw(), getPositions());

        if (!DriverStation.isAutonomous()) {
            for (Optional<EstimatedRobotPose> result : s_Vision.getEstimatedGlobalPoses(getPose())) {
                if (result.isPresent()) {
                    EstimatedRobotPose camPose = result.get();
                    swervePoseEstimator.addVisionMeasurement(
                            camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
                }
            }
        }

        field.setRobotPose(getPose());

        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
        Logger.getInstance().recordOutput("Odometry/RobotPose", getPose());
        Logger.getInstance().recordOutput("SwerveModuleStates", getStates());

        SmartDashboard.putNumber("Pigeon2 Yaw", getYaw().getDegrees());
        SmartDashboard.putNumber("Pigeon2 Pitch", getPitch().getDegrees());
        SmartDashboard.putNumber("Pigeon2 Roll", getRoll().getDegrees());
    }
}
