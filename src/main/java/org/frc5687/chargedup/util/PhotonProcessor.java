package org.frc5687.chargedup.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonProcessor {

    private static final int NUM_CAMERAS = 4;
    private final PhotonCamera _northWestCamera;
    private final PhotonCamera _southEastTopCamera;
    private final PhotonCamera _southWestCamera;
    private final PhotonCamera _southEastCamera;
    private final PhotonPoseEstimator _northWestCameraEstimator;
    private final PhotonPoseEstimator _southEastTopCameraEstimator;
    private final PhotonPoseEstimator _southWestCameraEstimator;
    private final PhotonPoseEstimator _southEastCameraEstimator;

    private final ExecutorService _executorService;
    public PhotonProcessor(AprilTagFieldLayout layout) {
        _northWestCamera = new PhotonCamera("North_East_Camera"); // name is wrong, really west camera.
        _southEastTopCamera = new PhotonCamera("South_East_Top_Camera");
        _southWestCamera = new PhotonCamera("South_West_Camera");
        _southEastCamera = new PhotonCamera("South_East_Camera");
        _executorService = Executors.newFixedThreadPool(NUM_CAMERAS);

        setPipeline(Pipeline.FAR);

        // dummy values
        Transform3d robotToNorthWestCam =
                new Transform3d(new Translation3d(-0.2, .14, 0.73), new Rotation3d(0, 0, 0));

        Transform3d robotToSouthEastTopCam =
                new Transform3d(new Translation3d(-0.25, -0.142, 0.655), new Rotation3d(0, 0, Units.degreesToRadians(170)));

        Transform3d robotToSouthWestCam =
                new Transform3d(
                        new Translation3d(-.21, .24, .442),
                        new Rotation3d(0, 0, Units.degreesToRadians(150)));

        Transform3d robotToSouthEastCam =
                new Transform3d(
                        new Translation3d(-.21, -.24, .442),
                        new Rotation3d(0, 0, Units.degreesToRadians(-150)));

        _northWestCameraEstimator =
                new PhotonPoseEstimator(
                        layout,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                        _northWestCamera,
                        robotToNorthWestCam);

        _southEastTopCameraEstimator =
                new PhotonPoseEstimator(
                        layout,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                        _southEastTopCamera,
                        robotToSouthEastTopCam);

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
        _northWestCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southEastTopCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southEastCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//        setLowestAmbiguity();
    }

    public void setPipeline(Pipeline pipeline) {
        // _northCamera.setPipelineIndex(pipeline.getValue());
        _southWestCamera.setPipelineIndex(pipeline.getValue());
        _southEastCamera.setPipelineIndex(pipeline.getValue());
    }

    public double getNorthWestCameraLatency() {
        return _northWestCamera.getLatestResult().getLatencyMillis();
    }

    public double getNorthEastCameraLatency() {
        return _southEastTopCamera.getLatestResult().getLatencyMillis();
    }

    public boolean hasNorthWestCameraTargets() {
        return _northWestCamera.getLatestResult().hasTargets();
    }

    public boolean hasNorthEastCameraTargets() {
        return _southEastTopCamera.getLatestResult().hasTargets();
    }

    public void setLowestAmbiguity() {
        _southEastCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southEastTopCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southWestCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _northWestCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getNorthWestCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _northWestCameraEstimator.setReferencePose(prevEstimatedPose);
        return _northWestCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getSouthEastTopCameraEstimatedGloablPose(
            Pose2d prevEstimatedPose) {
        _southEastTopCameraEstimator.setReferencePose(prevEstimatedPose);
        return _southEastTopCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getSouthWestCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _southWestCameraEstimator.setReferencePose(prevEstimatedPose);
        return _southWestCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getSouthEastCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _southEastCameraEstimator.setReferencePose(prevEstimatedPose);
        return _southEastCameraEstimator.update();
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getNorthWestCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getNorthWestCameraEstimatedGlobalPose(prevEstimatedPose));
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getSouthEastTopCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getSouthEastTopCameraEstimatedGloablPose(prevEstimatedPose));
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

    public void updatePoseEstimator(SwerveDrivePoseEstimator estimator, Rotation2d imuHeading) {

        // Call the camera pose estimation methods asynchronously and in parallel
        Pose2d estimatedRobotPose = estimator.getEstimatedPosition();
        List<CompletableFuture<Optional<EstimatedRobotPose>>> poseFutures = new ArrayList<>();
        poseFutures.add(getNorthWestCameraEstimatedGlobalPoseAsync(estimatedRobotPose));
        poseFutures.add(getSouthEastTopCameraEstimatedGlobalPoseAsync(estimatedRobotPose));
        poseFutures.add(getSouthWestCameraEstimatedGlobalPoseAsync(estimatedRobotPose));
        poseFutures.add(getSouthEastCameraEstimatedGlobalPoseAsync(estimatedRobotPose));
        CompletableFuture.allOf(poseFutures.toArray(new CompletableFuture[0]))
                .thenRunAsync(() -> {
                    // Once all pose estimation methods have completed, process the results
                    for (CompletableFuture<Optional<EstimatedRobotPose>> poseFuture : poseFutures) {
                        Optional<EstimatedRobotPose> pose = poseFuture.join();
                        if (pose.isPresent()) {
                            EstimatedRobotPose estimatedPose = pose.get();
//                            Pose2d correctedPose = estimatedPose.estimatedPose.toPose2d().se;
                            estimator.addVisionMeasurement(estimatedRobotPose, estimatedPose.timestampSeconds);
                        }
                    }
                }, _executorService);

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
