package org.frc5687.chargedup.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;

import java.util.ArrayList;
import java.util.Optional;

public class VisionProcessor {

    private final PhotonCamera _camera;
    private final RobotPoseEstimator _estimator;

    public VisionProcessor(String camName, AprilTagFieldLayout layout) {
        _camera = new PhotonCamera(camName);
        // dummy values
        Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0,0,0));

        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(_camera, robotToCam));
        _estimator = new RobotPoseEstimator(layout, RobotPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    }

    public double getLatency() {
        return _camera.getLatestResult().getLatencyMillis();
    }

    public boolean hasTargets() {
        return _camera.getLatestResult().hasTargets();
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedPose) {
        _estimator.setReferencePose(prevEstimatedPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = _estimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
}
