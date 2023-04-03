package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.VectorTools.CustomPhoton.PhotonPoseEstimator;
import frc.VectorTools.CustomPhoton.PhotonPoseEstimator.PoseStrategy;

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
        return result;
    }

    public double[] getCameraMatrixData() {
        return cameraInputs.cameraMatrixData;
    }

    public double[] getDistCoeffsData() {
        return cameraInputs.distCoeffsData;
    }
}
