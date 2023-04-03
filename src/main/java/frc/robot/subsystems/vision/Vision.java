package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.VectorTools.CustomPhoton.PhotonPoseEstimator;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private ArrayList<Camera> cameras;
    private AprilTagFieldLayout aprilTagLayout;

    /**
     * Sets up the pose estimators and AprilTag layout.
     */
    public Vision(CameraIO leftCamera, CameraIO rightCamera) {
        this.cameras = new ArrayList<>();

        try {
            aprilTagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

            cameras.add(new Camera(leftCamera, Constants.PhotonVision.leftCameraPosition, aprilTagLayout));
            cameras.add(new Camera(rightCamera, Constants.PhotonVision.rightCameraPosition, aprilTagLayout));
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        }
    }

    /**
     * Returns a list of estimated global poses for each pose estimator.
     */
    public List<Optional<EstimatedRobotPose>> getEstimatedGlobalPoses(Pose2d prevEstimatedRobotPose) {
        ArrayList<Optional<EstimatedRobotPose>> robotPoses = new ArrayList<>();

        for (Camera camera : cameras) {
            PhotonPoseEstimator positionEstimation = camera.getPoseEstimator();

            if (positionEstimation == null) {
                robotPoses.add(Optional.empty());
            } else {
                positionEstimation.setReferencePose(prevEstimatedRobotPose);

                Optional<EstimatedRobotPose> estimatedPosition = positionEstimation.update(
                        camera.getPhotonPipelineResult(),
                        camera.getCameraMatrixData(),
                        camera.getDistCoeffsData());

                if (estimatedPosition.isPresent())
                    Logger.getInstance().recordOutput("Odometry/" + camera.cameraName,
                            estimatedPosition.get().estimatedPose);
                robotPoses.add(estimatedPosition);
            }
        }

        return robotPoses;
    }

    @Override
    public void periodic() {
        for (Camera camera : cameras) {
            camera.periodic();
        }
    }

    /**
     * Update the origin of pose based on alliance
     */
    public void updatePoseAlliance() {
        // Sets the april tag positions depending on which side the robot starts on.
        for (Camera camera : cameras) {
            PhotonPoseEstimator positionEstimation = camera.getPoseEstimator();
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                positionEstimation.getFieldTags().setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            } else {
                positionEstimation.getFieldTags().setOrigin(OriginPosition.kRedAllianceWallRightSide);
            }
        }
    }
}
