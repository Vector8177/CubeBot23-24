package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.VectorTools.CustomPhoton.PhotonPoseEstimator;
import frc.VectorTools.CustomPhoton.PhotonPoseEstimator.PoseStrategy;
import frc.VectorTools.util.PoseMeasurement;
import frc.robot.Constants;

public class Camera {
    private final CameraIO cameraIO;
    public final String cameraName;

    private final CameraIOInputsAutoLogged cameraInputs = new CameraIOInputsAutoLogged();

    private final Transform3d cameraPosition;
    private final AprilTagFieldLayout aprilTagLayout;
    private final PhotonPoseEstimator poseEstimator;

    public Camera(CameraIO camera, Transform3d cameraPosition, AprilTagFieldLayout aprilTagLayout) {
        this.cameraIO = camera;
        this.cameraName = cameraInputs.cameraName;
        this.cameraPosition = cameraPosition;
        this.aprilTagLayout = aprilTagLayout;

        poseEstimator = generatePoseEstimator();
    }

    private PhotonPoseEstimator generatePoseEstimator() {
        PhotonPoseEstimator positionEstimation = new PhotonPoseEstimator(aprilTagLayout,
                PoseStrategy.MULTI_TAG_PNP,
                cameraPosition);

        positionEstimation.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        return positionEstimation;
    }

    public void periodic() {
        cameraIO.updateInputs(cameraInputs);
        Logger.getInstance().processInputs("Cameras/" + cameraName, cameraInputs);
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public PhotonPipelineResult getPhotonPipelineResult() {
        PhotonPipelineResult result = new PhotonPipelineResult();
        result.createFromPacket(new Packet(cameraInputs.targetData));
        result.setTimestampSeconds(cameraInputs.targetTimestamp);
        return result;
    }

    public Optional<PoseMeasurement.Measurement> getEstimatedPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);

        return poseEstimator.update(
                getPhotonPipelineResult(),
                getCameraMatrixData(),
                getDistCoeffsData()).flatMap((result) -> {
                    if (result.targetsUsed.get(0).getBestCameraToTarget().getTranslation()
                            .getNorm() > Constants.PoseEstimation.POSE_DISTANCE_CUTOFF
                            || result.targetsUsed.get(0)
                                    .getPoseAmbiguity() > Constants.PoseEstimation.POSE_AMBIGUITY_CUTOFF) {
                        return Optional.empty();
                    }

                    // Reject pose estimates outside the field
                    if (result.estimatedPose.toPose2d().getX() < 0
                            || result.estimatedPose.toPose2d().getX() > Constants.FieldConstants.fieldLength ||
                            result.estimatedPose.toPose2d().getY() < 0
                            || result.estimatedPose.toPose2d().getY() > Constants.FieldConstants.fieldWidth) {
                        return Optional.empty();
                    }

                    Logger.getInstance().recordOutput("Odometry/" + cameraName + "/RobotPose",
                            result.estimatedPose);

                    return Optional.of(new PoseMeasurement.Measurement(
                            result.timestampSeconds,
                            result.estimatedPose,
                            Constants.PoseEstimation.PHOTON_VISION_STD_DEV.forMeasurement(
                                    result.targetsUsed.get(0).getBestCameraToTarget().getX(),
                                    result.targetsUsed.size())));
                });
    }

    public double[] getCameraMatrixData() {
        return cameraInputs.cameraMatrixData;
    }

    public double[] getDistCoeffsData() {
        return cameraInputs.distCoeffsData;
    }
}
