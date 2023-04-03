package frc.robot.subsystems.vision;

import frc.VectorTools.CustomPhoton.PhotonCameraRaw;

public class CameraIOPhoton implements CameraIO {
    private String cameraName;
    private PhotonCameraRaw camera;

    public CameraIOPhoton(String cameraName) {
        this.cameraName = cameraName;
        camera = new PhotonCameraRaw(cameraName);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.cameraName = cameraName;
        inputs.connected = camera.isConnected();
        inputs.driverMode = camera.getDriverMode();
        inputs.byteArray = camera.getRawBytes();
        inputs.cameraMatrixData = camera.getRawCameraMatrix();
        inputs.distCoeffsData = camera.getRawDistCoeffs();
    }

}
