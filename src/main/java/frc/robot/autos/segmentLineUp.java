package frc.robot.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class segmentLineUp {

    /**
     * 
     * @param s_Swerve
     * @param segment    the segment TODO
     * @param startPoint
     */
    public static PathPlannerTrajectory getTrajectory(Constants.SEGMENT segment, Supplier<PathPoint> startPoint) {
        PathPoint lineUpPoint = startPoint.get();

        switch (segment) {
            case CONE_1:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.98, 4.93),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
                break;
            case CONE_2:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.98, 3.89),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
                break;
            case CONE_3:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.98, 3.25),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
                break;
            case CONE_4:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.98, 2.2),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
                break;
            case CONE_5:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.98, 1.6),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
                break;
            case CONE_6:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.98, .47),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
                break;
            case CUBE_1:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.98, 4.43),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
                break;
            case CUBE_2:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.98, 2.74),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
                break;
            case CUBE_3:
                lineUpPoint = new PathPoint(
                        new Translation2d(1.98, 1.05),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180));
                break;
        }

        PathPlannerTrajectory trajectoryToSegment = PathPlanner.generatePath(
                Constants.Autonomous.constraints,
                startPoint.get(),
                lineUpPoint);

        return trajectoryToSegment;
    }
}
