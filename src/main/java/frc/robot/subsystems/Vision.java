package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private ArrayList<PhotonPoseEstimator> poseEstimators;
    private AprilTagFieldLayout aprilTagLayout;

    /**
     * Sets up the pose estimators and AprilTag layout.
     */
    public Vision() {
        poseEstimators = new ArrayList<>();
        PhotonCamera leftCamera = new PhotonCamera(Constants.PhotonVision.leftCameraName);
        PhotonCamera rightCamera = new PhotonCamera(Constants.PhotonVision.rightCameraName);

        try {
            aprilTagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

            poseEstimators.add(generatePoseEstimator(leftCamera, Constants.PhotonVision.leftCameraPosition));
            poseEstimators.add(generatePoseEstimator(rightCamera, Constants.PhotonVision.rightCameraPosition));
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        }
    }

    /**
     * Generates a pose estimator for a camera.
     */
    public PhotonPoseEstimator generatePoseEstimator(PhotonCamera camera, Transform3d cameraPosition) {
        PhotonPoseEstimator positionEstimation = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP,
                camera,
                cameraPosition);

        positionEstimation.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        return positionEstimation;
    }

    /**
     * Returns a list of estimated global poses for each pose estimator.
     */
    public List<Optional<EstimatedRobotPose>> getEstimatedGlobalPoses(Pose2d prevEstimatedRobotPose) {
        ArrayList<Optional<EstimatedRobotPose>> robotPoses = new ArrayList<>();

        for (PhotonPoseEstimator positionEstimation : poseEstimators) {
            if (positionEstimation == null) {
                robotPoses.add(Optional.empty());
            } else {
                positionEstimation.setReferencePose(prevEstimatedRobotPose);
                robotPoses.add(positionEstimation.update());
            }
        }

        return robotPoses;
    }

    /**
     * Update the origin of pose based on alliance
     */
    public void updatePoseAlliance() {
        // Sets the april tag positions depending on which side the robot starts on.
        for (PhotonPoseEstimator positionEstimation : poseEstimators) {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                positionEstimation.getFieldTags().setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            } else {
                positionEstimation.getFieldTags().setOrigin(OriginPosition.kRedAllianceWallRightSide);
            }
        }
    }

}