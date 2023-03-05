package org.frc5687.chargedup.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonProcessor {

    private final PhotonCamera _northCamera;
    private final PhotonCamera _southWestCamera;
    private final PhotonCamera _southEastCamera;
    private final PhotonPoseEstimator _northCameraEstimator;
    private final PhotonPoseEstimator _southWestCameraEstimator;
    private final PhotonPoseEstimator _southEastCameraEstimator;

    public PhotonProcessor(AprilTagFieldLayout layout) {
        _northCamera = new PhotonCamera("North_Camera");
        _southWestCamera = new PhotonCamera("South_West_Camera");
        _southEastCamera = new PhotonCamera("South_East_Camera");

        setPipeline(Pipeline.FAR);

        // dummy values
        Transform3d robotToNorthCam =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(0.5), Units.inchesToMeters(4.5), Units.inchesToMeters(20)),
                        new Rotation3d(0, 0, 0));

        Transform3d robotToSouthWestCam =
                new Transform3d(
                        new Translation3d(
                                -.243,
                                .249,
                                .442),
                        new Rotation3d(0, 0, Units.degreesToRadians(140)));

        Transform3d robotToSouthEastCam =
                new Transform3d(
                        new Translation3d(
                                -.243,
                                -.249,
                                .442),
                        new Rotation3d(0, 0, Units.degreesToRadians(-140)));

        _northCameraEstimator =
                new PhotonPoseEstimator(
                        layout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, _northCamera, robotToNorthCam);

        _southWestCameraEstimator =
                new PhotonPoseEstimator(
                        layout,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                        _southWestCamera,
                        robotToSouthWestCam);

        _southEastCameraEstimator =
                new PhotonPoseEstimator(
                        layout,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                        _southEastCamera,
                        robotToSouthEastCam);

        _southWestCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _northCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southEastCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
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

    public Optional<EstimatedRobotPose> getSouthWestCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        return _southWestCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getSouthEastCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        return _southEastCameraEstimator.update();
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getNorthCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getNorthCameraEstimatedGlobalPose(prevEstimatedPose));
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getSouthWestCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getSouthWestCameraEstimatedGlobalPose(prevEstimatedPose));
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getSouthEastCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getSouthEastCameraEstimatedGlobalPose(prevEstimatedPose));
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
