package org.frc5687.chargedup.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.lib.vision.TrackedObjectInfo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.RobotPoseEstimator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class PhotonProcessor {

    private final PhotonCamera _northCamera;
    private final PhotonCamera _southWestCamera;
    private final PhotonCamera _southEastCamera;
    private final PhotonPoseEstimator _northCameraEstimator;
//    private final PhotonPoseEstimator _southWestCameraEstimator;
//    private final PhotonPoseEstimator _southEastCameraEstimator;

    public PhotonProcessor(AprilTagFieldLayout layout) {
        _northCamera = new PhotonCamera("North_Camera");
        _southWestCamera = new PhotonCamera("South_West_Camera");
        _southEastCamera = new PhotonCamera("South_East_Camera");

        setPipeline(Pipeline.FAR);

        // dummy values
        Transform3d robotToNorthCam = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(0.5),
                        Units.inchesToMeters(4.5),
                        Units.inchesToMeters(20)
                ),
                new Rotation3d(0,0,0));

//        camList.add(new Pair<PhotonCamera, Transform3d>(_camera, robotToCam));

        _northCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                _northCamera,
                robotToNorthCam
        );
    }


    public void setPipeline(Pipeline pipeline) {
        _northCamera.setPipelineIndex(pipeline.getValue());
        _southWestCamera.setPipelineIndex(pipeline.getValue());
        _southEastCamera.setPipelineIndex(pipeline.getValue());
    }
    public double getNorthCameraLatency() {
        return _northCamera.getLatestResult().getLatencyMillis();
    }

    public boolean hasNorthCameraTargets() {
        return _northCamera.getLatestResult().hasTargets();
    }

    public Optional<EstimatedRobotPose> getNorthCameraEstimatedGlobalPose(Pose2d prevEstimatedPose) {
//        _northCameraEstimator.setReferencePose(prevEstimatedPose);
        return _northCameraEstimator.update();
    }


    public enum Pipeline {
        FAR(0),
        CLOSE(1);

        private final int _value;

        Pipeline(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
