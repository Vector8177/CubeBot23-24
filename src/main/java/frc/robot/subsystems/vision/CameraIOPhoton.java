package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;

public class CameraIOPhoton implements CameraIO {
    private String cameraName;
    private PhotonCamera camera;

    public CameraIOPhoton(String cameraName) {
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.cameraName = cameraName;
        inputs.connected = camera.isConnected();
        inputs.driverMode = camera.getDriverMode();
        inputs.targetData = camera.getLatestResult().populatePacket(new Packet(new byte[] {})).getData();
        inputs.targetTimestamp = camera.getLatestResult().getTimestampSeconds();
        inputs.cameraMatrixData = camera.getCameraMatrix().isPresent() ? camera.getCameraMatrix().get().getData()
                : new double[] {};
        inputs.distCoeffsData = camera.getDistCoeffs().isPresent() ? camera.getDistCoeffs().get().getData()
                : new double[] {};
    }

}
