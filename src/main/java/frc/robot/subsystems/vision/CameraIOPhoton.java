package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOPhoton implements CameraIO {
    private PhotonCamera camera;

    public CameraIOPhoton(String cameraName) {
        camera = new PhotonCamera(cameraName);

        camera.setDriverMode(false);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.cameraName = camera.getName();
        inputs.connected = camera.isConnected();
        inputs.driverMode = camera.getDriverMode();
        PhotonPipelineResult latestResult = camera.getLatestResult();
        inputs.targetData =
                latestResult.populatePacket(new Packet(latestResult.getPacketSize())).getData();
        inputs.targetTimestamp = camera.getLatestResult().getTimestampSeconds();
        inputs.cameraMatrixData =
                camera.getCameraMatrix().isPresent()
                        ? camera.getCameraMatrix().get().getData()
                        : new double[] {};
        inputs.distCoeffsData =
                camera.getDistCoeffs().isPresent()
                        ? camera.getDistCoeffs().get().getData()
                        : new double[] {};
    }
}
