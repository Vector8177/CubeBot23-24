package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.common.dataflow.structures.Packet;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.VectorTools.CustomPhoton.PhotonPoseEstimatorRaw;
import frc.VectorTools.CustomPhoton.PhotonPoseEstimatorRaw.PoseStrategy;

public class Camera {
    private final CameraIO cameraIO;
    public final String cameraName;

    private final CameraIOInputsAutoLogged cameraInputs = new CameraIOInputsAutoLogged();

    private final Transform3d cameraPosition;
    private final AprilTagFieldLayout aprilTagLayout;
    private final PhotonPoseEstimatorRaw poseEstimator;

    public Camera(CameraIO camera, Transform3d cameraPosition, AprilTagFieldLayout aprilTagLayout) {
        this.cameraIO = camera;
        this.cameraName = cameraInputs.cameraName;
        this.cameraPosition = cameraPosition;
        this.aprilTagLayout = aprilTagLayout;

        poseEstimator = generatePoseEstimator();
    }

    private PhotonPoseEstimatorRaw generatePoseEstimator() {
        PhotonPoseEstimatorRaw positionEstimation = new PhotonPoseEstimatorRaw(aprilTagLayout,
                PoseStrategy.MULTI_TAG_PNP,
                cameraPosition);

        positionEstimation.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        return positionEstimation;
    }

    public void periodic() {
        cameraIO.updateInputs(cameraInputs);
        Logger.getInstance().processInputs("Cameras/" + cameraName, cameraInputs);
    }

    public PhotonPoseEstimatorRaw getPoseEstimator() {
        return poseEstimator;
    }

    public Packet getPacket() {
        return new Packet(cameraInputs.byteArray);
    }

    public double[] getCameraMatrixData() {
        return cameraInputs.cameraMatrixData;
    }

    public double[] getDistCoeffsData() {
        return cameraInputs.distCoeffsData;
    }
}
