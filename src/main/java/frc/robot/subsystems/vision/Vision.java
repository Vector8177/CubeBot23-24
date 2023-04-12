package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.VectorTools.CustomPhoton.PhotonPoseEstimator;
import frc.VectorTools.util.PoseMeasurement;
import frc.VectorTools.util.PoseMeasurement.Measurement;

public class Vision extends SubsystemBase {
    private ArrayList<Camera> cameras;
    private AprilTagFieldLayout aprilTagLayout;

    /**
     * Sets up the cameras and AprilTag layout.
     */
    public Vision(CameraIO leftCamera, CameraIO rightCamera) {
        this.cameras = new ArrayList<>();

        try {
            aprilTagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

            cameras.add(new Camera(leftCamera, VisionConstants.leftCameraPosition, aprilTagLayout));
            cameras.add(new Camera(rightCamera, VisionConstants.rightCameraPosition, aprilTagLayout));
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        }
    }

    /**
     * Returns a list of estimated global poses for each pose estimator.
     */
    public List<Optional<PoseMeasurement.Measurement>> getEstimatedGlobalPoses(Pose2d prevEstimatedRobotPose) {
        ArrayList<Optional<Measurement>> robotPoses = new ArrayList<>();

        for (Camera camera : cameras) {
            Optional<Measurement> measurement = camera.getEstimatedPose(prevEstimatedRobotPose);
            robotPoses.add(measurement);
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
