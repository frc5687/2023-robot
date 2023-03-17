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
    private final PhotonCamera _northEastCamera;
    private final PhotonCamera _southWestCamera;
    private final PhotonCamera _southEastCamera;
    private final PhotonPoseEstimator _northWestCameraEstimator;
    private final PhotonPoseEstimator _northEastCameraEstimator;
    private final PhotonPoseEstimator _southWestCameraEstimator;
    private final PhotonPoseEstimator _southEastCameraEstimator;

    private final ExecutorService _executorService;
    public PhotonProcessor(AprilTagFieldLayout layout) {
        _northWestCamera = new PhotonCamera("North_West_Camera");
        _northEastCamera = new PhotonCamera("North_East_Camera");
        _southWestCamera = new PhotonCamera("South_West_Camera");
        _southEastCamera = new PhotonCamera("South_East_Camera");
        _executorService = Executors.newFixedThreadPool(NUM_CAMERAS);

        setPipeline(Pipeline.FAR);

        // dummy values
        Transform3d robotToNorthWestCam =
                new Transform3d(
                        new Translation3d(
                                0.73, -0.14, 0.2),
                        new Rotation3d(0, 0, 0));

        Transform3d robotToNorthEastCam =
                new Transform3d(
                        new Translation3d(
                               0.62, 0.14, 0.03),
                        new Rotation3d(0, 0, 0));

        Transform3d robotToSouthWestCam =
                new Transform3d(
                        new Translation3d(
                                -.243,
                                .249,
                                .442),
                        new Rotation3d(0, 0, Units.degreesToRadians(-140)));

        Transform3d robotToSouthEastCam =
                new Transform3d(
                        new Translation3d(
                                -.243,
                                -.249,
                                .442),
                        new Rotation3d(0, 0, Units.degreesToRadians(140)));

        _northWestCameraEstimator =
                new PhotonPoseEstimator(
                        layout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, _northWestCamera, robotToNorthWestCam);

        _northEastCameraEstimator =
                new PhotonPoseEstimator(
                        layout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, _northEastCamera, robotToNorthEastCam);

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
        _northEastCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southEastCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
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
        return _northEastCamera.getLatestResult().getLatencyMillis();
    }

    public boolean hasNorthWestCameraTargets() {
        return _northWestCamera.getLatestResult().hasTargets();
    }

    public boolean hasNorthEastCameraTargets() {
        return _northEastCamera.getLatestResult().hasTargets();
    }

    public Optional<EstimatedRobotPose> getNorthWestCameraEstimatedGlobalPose(Pose2d prevEstimatedPose) {
        //        _northCameraEstimator.setReferencePose(prevEstimatedPose);
        return _northWestCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getNorthEastCameraEstimatedGlobalPose(Pose2d prevEstimatedPose) {
        //        _northCameraEstimator.setReferencePose(prevEstimatedPose);
        return _northEastCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getSouthWestCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        return _southWestCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getSouthEastCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        return _southEastCameraEstimator.update();
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getNorthWestCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getNorthWestCameraEstimatedGlobalPose(prevEstimatedPose));
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getNorthEastCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getNorthEastCameraEstimatedGlobalPose(prevEstimatedPose));
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
        poseFutures.add(getNorthEastCameraEstimatedGlobalPoseAsync(estimatedRobotPose));
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
