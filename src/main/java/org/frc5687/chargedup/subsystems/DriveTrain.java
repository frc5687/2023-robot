/* Team 5687 (C)2020-2022 */
package org.frc5687.chargedup.subsystems;

import static org.frc5687.chargedup.Constants.DriveTrain.*;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.*;
import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.lib.swerve.SwerveSetpoint;
import org.frc5687.lib.swerve.SwerveSetpointGenerator;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frc5687.lib.vision.TrackedObjectInfo;
import org.frc5687.lib.vision.VisionProcessor;
import org.photonvision.EstimatedRobotPose;

public class DriveTrain extends OutliersSubsystem {
    // Order we define swerve modules in kinematics
    private static final int NORTH_WEST_IDX = 0;
    private static final int SOUTH_WEST_IDX = 1;
    private static final int SOUTH_EAST_IDX = 2;
    private static final int NORTH_EAST_IDX = 3;
    private final DiffSwerveModule[] _modules;
    private final SwerveDriveKinematics _kinematics;
    private final SwerveDriveOdometry _odometry;
    private final SwerveDrivePoseEstimator _poseEstimator;
    // module sensor readings we would like to read all at once over CAN
    private final BaseStatusSignalValue[] _moduleSignals;
    // controllers [Heading, Pose, Trajectory]
    private ControlState _controlState;
    private final SwerveHeadingController _headingController;
    private final HolonomicDriveController _poseController;
    private final PPHolonomicDriveController _trajectoryController;

    private boolean _isRedAlliance = false;
    // IMU (Pigeon)
    private final Pigeon2 _imu;
    private double _yawOffset;



    // Setpoint generator for swerve.
    private final SwerveSetpointGenerator _swerveSetpointGenerator;
    private KinematicLimits _kinematicLimits = KINEMATIC_LIMITS;

    private final SystemIO _systemIO;

    // Vision Processors
    private final VisionProcessor _visionProcessor;
    private final PhotonProcessor _photonProcessor;

    private final Field2d _field;
    private Mode _mode = Mode.NORMAL;
    private Pose2d _hoverGoal;

    public DriveTrain(
            OutliersContainer container,
            VisionProcessor processor,
            PhotonProcessor photonProcessor,
            Pigeon2 imu) {
        super(container);

        _visionProcessor = processor;
        _photonProcessor = photonProcessor;

        // configure our system IO and pigeon;
        _imu = imu;
        _systemIO = new SystemIO();
        _isRedAlliance = DriverStation.getAlliance() == Alliance.Red;

        // This should set the Pigeon to 0.
        // set update frequency to 200hz
        _imu.getYaw().setUpdateFrequency(200);
        _imu.getPitch().setUpdateFrequency(200);
        _yawOffset = _imu.getYaw().getValue();
        readIMU();

        _controlState = ControlState.NEUTRAL;

        // set up the modules
        _modules = new DiffSwerveModule[4];
        _modules[NORTH_WEST_IDX] =
                new DiffSwerveModule(
                        NORTH_WEST_CONFIG,
                        RobotMap.CAN.TALONFX.NORTH_WEST_OUTER,
                        RobotMap.CAN.TALONFX.NORTH_WEST_INNER,
                        RobotMap.DIO.ENCODER_NW);
        _modules[SOUTH_WEST_IDX] =
                new DiffSwerveModule(
                        SOUTH_WEST_CONFIG,
                        RobotMap.CAN.TALONFX.SOUTH_WEST_OUTER,
                        RobotMap.CAN.TALONFX.SOUTH_WEST_INNER,
                        RobotMap.DIO.ENCODER_SW);
        _modules[SOUTH_EAST_IDX] =
                new DiffSwerveModule(
                        SOUTH_EAST_CONFIG,
                        RobotMap.CAN.TALONFX.SOUTH_EAST_INNER,
                        RobotMap.CAN.TALONFX.SOUTH_EAST_OUTER,
                        RobotMap.DIO.ENCODER_SE);
        _modules[NORTH_EAST_IDX] =
                new DiffSwerveModule(
                        NORTH_EAST_CONFIG,
                        RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                        RobotMap.CAN.TALONFX.NORTH_EAST_OUTER,
                        RobotMap.DIO.ENCODER_NE);

        // This should set the Pigeon to 0.
        _imu.getYaw().setUpdateFrequency(200);
        _imu.getPitch().setUpdateFrequency(200);
        _yawOffset = _imu.getYaw().getValue();
        readIMU();

        _controlState = ControlState.MANUAL;

        _kinematics =
                new SwerveDriveKinematics(
                        _modules[NORTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_EAST_IDX].getModuleLocation(),
                        _modules[NORTH_EAST_IDX].getModuleLocation());
        _odometry =
                new SwerveDriveOdometry(
                        _kinematics,
                        getHeading(),
                        new SwerveModulePosition[] {
                            _modules[NORTH_WEST_IDX].getModulePosition(),
                            _modules[SOUTH_WEST_IDX].getModulePosition(),
                            _modules[SOUTH_EAST_IDX].getModulePosition(),
                            _modules[NORTH_EAST_IDX].getModulePosition()
                        },
                        new Pose2d(0, 0, getHeading()));

        _poseEstimator =
                new SwerveDrivePoseEstimator(
                        _kinematics,
                        getHeading(),
                        new SwerveModulePosition[] {
                            _modules[NORTH_WEST_IDX].getModulePosition(),
                            _modules[SOUTH_WEST_IDX].getModulePosition(),
                            _modules[SOUTH_EAST_IDX].getModulePosition(),
                            _modules[NORTH_EAST_IDX].getModulePosition()
                        },
                        new Pose2d(0, 0, getHeading()),
                        VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1)),
                        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(70)));
        _swerveSetpointGenerator =
                new SwerveSetpointGenerator(
                        _kinematics,
                        new Translation2d[] {
                            _modules[NORTH_WEST_IDX].getModuleLocation(),
                            _modules[SOUTH_WEST_IDX].getModuleLocation(),
                            _modules[SOUTH_EAST_IDX].getModuleLocation(),
                            _modules[NORTH_EAST_IDX].getModuleLocation()
                        });

        // controllers
        _headingController = new SwerveHeadingController(Constants.UPDATE_PERIOD);

        _poseController =
                new HolonomicDriveController(
                        new PIDController(
                                Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                        new PIDController(
                                Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                        new ProfiledPIDController(
                                MAINTAIN_kP,
                                MAINTAIN_kI,
                                MAINTAIN_kP,
                                new TrapezoidProfile.Constraints(
                                        Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                        Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));

        _trajectoryController = new PPHolonomicDriveController(
                new PIDController(kP, kI, kD),
                new PIDController(kP, kI, kD),
                new PIDController(ANGLE_TRAJECTORY_kP, ANGLE_TRAJECTORY_kI, ANGLE_TRAJECTORY_kD)
        );

        // module CAN bus sensor outputs (position, velocity of each motor) all of them are called once per loop at the start.
        _moduleSignals = new BaseStatusSignalValue[NUM_MODULES * 4];
        for (int i = 0; i < NUM_MODULES; ++i) {
            var signals = _modules[i].getSignals();
            _moduleSignals[(i * 4)] = signals[0];
            _moduleSignals[(i * 4) + 1] = signals[1];
            _moduleSignals[(i * 4) + 2] = signals[2];
            _moduleSignals[(i * 4) + 3] = signals[3];
        }

        _field = new Field2d();
        _hoverGoal = new Pose2d();

        zeroGyroscope();
        resetRobotPose(new Pose2d());

        readModules();
        setSetpointFromMeasuredModules();
    }

    /**
     * SystemIO is input, output of our drivetrain that we want to cache.
     */
    public static class SystemIO {
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measuredStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
                };
        SwerveModulePosition[] measuredPositions =
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                };

        Rotation2d heading = new Rotation2d(0.0);
        double pitch = 0.0;
        // outputs
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[4]);
    }

    // DriveTrain periodic functions, these run constantly even if there is no command scheduled or the robot is disabled.
    public void modulePeriodic() {
        // use for modules as controller is running at 200Hz.
        BaseStatusSignalValue.waitForAll(0.0, _moduleSignals);
        for (DiffSwerveModule diffSwerveModule : _modules) {
            diffSwerveModule.controlPeriodic();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        readIMU();
        readModules();
        updateDesiredStates();
        setModuleStates(_systemIO.setpoint.moduleStates);
    }

    @Override
    public void dataPeriodic(double timestamp) {
        _poseEstimator.update(getHeading(), _systemIO.measuredPositions);
        if (_imu.getPitch().getValue() < 5){
        Pose2d prevEstimatedPose = _poseEstimator.getEstimatedPosition();
            CompletableFuture<Optional<EstimatedRobotPose>> northWestPoseFuture =
                    _photonProcessor.getNorthWestCameraEstimatedGlobalPoseAsync(prevEstimatedPose);
            CompletableFuture<Optional<EstimatedRobotPose>> northEastPoseFuture =
                    _photonProcessor.getSouthEastTopCameraEstimatedGlobalPoseAsync(prevEstimatedPose);
            CompletableFuture<Optional<EstimatedRobotPose>> southWestPoseFuture =
                    _photonProcessor.getSouthWestCameraEstimatedGlobalPoseAsync(prevEstimatedPose);
            CompletableFuture<Optional<EstimatedRobotPose>> southEastPoseFuture =
                    _photonProcessor.getSouthEastCameraEstimatedGlobalPoseAsync(prevEstimatedPose);

            Optional<EstimatedRobotPose> northWestPose = northWestPoseFuture.join();
            Optional<EstimatedRobotPose> northEastPose = northEastPoseFuture.join();
            Optional<EstimatedRobotPose> southWestPose = southWestPoseFuture.join();
            Optional<EstimatedRobotPose> southEastPose = southEastPoseFuture.join();
            if (northWestPose.isPresent()) {
                EstimatedRobotPose camNorthWestPose = northWestPose.get();
                _poseEstimator.addVisionMeasurement(
                        camNorthWestPose.estimatedPose.toPose2d(), camNorthWestPose.timestampSeconds);
            }
            if (northEastPose.isPresent()) {
                EstimatedRobotPose camNorthEastPose = northEastPose.get();
                _poseEstimator.addVisionMeasurement(
                        camNorthEastPose.estimatedPose.toPose2d(), camNorthEastPose.timestampSeconds);
            }
            if (southWestPose.isPresent()) {
                EstimatedRobotPose camSW = southWestPose.get();
                _poseEstimator.addVisionMeasurement(camSW.estimatedPose.toPose2d(), camSW.timestampSeconds);
            }
            if (southEastPose.isPresent()) {
                EstimatedRobotPose camSE = southEastPose.get();
                _poseEstimator.addVisionMeasurement(camSE.estimatedPose.toPose2d(), camSE.timestampSeconds);
            }
            _field.setRobotPose(_poseEstimator.getEstimatedPosition());
        }
    }

    // Heading controller functions
    public HeadingState getHeadingControllerState() {
        return _headingController.getHeadingState();
    }

    public void setHeadingControllerState(HeadingState state) {
        _headingController.setState(state);
    }

    public double getRotationCorrection() {
        return _headingController.getRotationCorrection(getHeading());
    }
    public void temporaryDisabledHeadingController() {
        _headingController.temporaryDisable();
    }

    public void disableHeadingController() {
        _headingController.disable();
    }

    public void initializeHeadingController() {
        _headingController.setMaintainHeading(getHeading());
    }

    public void incrementHeadingControllerAngle() {
        Rotation2d heading = getHeading();
        _headingController.setMaintainHeading(
                Rotation2d.fromDegrees(heading.getDegrees() + Constants.DriveTrain.BUMP_DEGREES));
    }

    public void decrementHeadingControllerAngle() {
        Rotation2d heading = getHeading();
        _headingController.setMaintainHeading(
                Rotation2d.fromDegrees(heading.getDegrees() - Constants.DriveTrain.BUMP_DEGREES));
    }

    public void setSnapHeading(Rotation2d heading) {
        _headingController.setSnapHeading(heading);
    }
    public void setMaintainHeading(Rotation2d heading) {
        _headingController.setMaintainHeading(heading);
    }

    public void plotTrajectory(Trajectory t, String name) {
        _field.getObject(name).setTrajectory(t);
    }
    public void startModules() {
        for (DiffSwerveModule diffSwerveModule : _modules) {
            diffSwerveModule.start();
        }
    }

    public void readModules() {
        for (int module = 0; module < _modules.length; module++) {
            _systemIO.measuredStates[module] = _modules[module].getState();
            _systemIO.measuredPositions[module] = _modules[module].getModulePosition();
        }
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int module = 0; module < _modules.length; module++) {
            _modules[module].setIdealState(states[module]);
        }
    }

    public void setSetpointFromMeasuredModules() {
        System.arraycopy(
                _systemIO.measuredStates, 0, _systemIO.setpoint.moduleStates, 0, _modules.length);
        _systemIO.setpoint.chassisSpeeds = _kinematics.toChassisSpeeds(_systemIO.setpoint.moduleStates);
    }

    public void orientModules(Rotation2d moduleAngle) {
        for (int module = 0; module < _modules.length; module++) {
            _systemIO.setpoint.moduleStates[module] = new SwerveModuleState(0.0, moduleAngle);
        }
    }

    public void setControlState(ControlState state) {
        _controlState = state;
    }

    // Swerve setpoint generator functions
    public ControlState getControlState() {
        return _controlState;
    }

    public SwerveSetpoint getSetpoint() {
        return _systemIO.setpoint;
    }

    public void setKinematicLimits(KinematicLimits limits) {
        if (limits != _kinematicLimits) {
            _kinematicLimits = limits;
        }
    }

    public void updateDesiredStates() {
        Pose2d robotPoseVel =
                new Pose2d(
                        _systemIO.desiredChassisSpeeds.vxMetersPerSecond * Constants.UPDATE_PERIOD,
                        _systemIO.desiredChassisSpeeds.vyMetersPerSecond * Constants.UPDATE_PERIOD,
                        Rotation2d.fromRadians(
                                _systemIO.desiredChassisSpeeds.omegaRadiansPerSecond * Constants.UPDATE_PERIOD));

        Twist2d twistVel = new Pose2d().log(robotPoseVel);
        ChassisSpeeds updatedChassisSpeeds =
                new ChassisSpeeds(
                        twistVel.dx / Constants.UPDATE_PERIOD,
                        twistVel.dy / Constants.UPDATE_PERIOD,
                        twistVel.dtheta / Constants.UPDATE_PERIOD);
        _systemIO.setpoint =
                _swerveSetpointGenerator.generateSetpoint(
                        _kinematicLimits,
                        _systemIO.setpoint,
                        //                        _systemIO.desiredChassisSpeeds,
                        updatedChassisSpeeds,
                        Constants.UPDATE_PERIOD);
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        _systemIO.desiredChassisSpeeds = chassisSpeeds;
    }

    public void setVelocityPose(Pose2d pose, boolean isShooter) {
        ChassisSpeeds speeds =
                _poseController.calculate(
                        _poseEstimator.getEstimatedPosition(), pose, 0.0, _systemIO.heading);
        _headingController.setMaintainHeading(isShooter ? new Rotation2d(Math.PI) : new Rotation2d());
        speeds.omegaRadiansPerSecond = _headingController.getRotationCorrection(getHeading());
        _systemIO.desiredChassisSpeeds = speeds;
    }


    public void followTrajectory(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds speeds =
                _poseController.calculate(
                        getOdometryPose(), goal, _systemIO.heading);
        _headingController.setMaintainHeading(heading);
        speeds.omegaRadiansPerSecond = _headingController.getRotationCorrection(getHeading());
        _systemIO.desiredChassisSpeeds = speeds;
    }

    public void followTrajectory(PathPlannerTrajectory.PathPlannerState desiredState) {
        ChassisSpeeds speeds = _trajectoryController.calculate(getEstimatedPose(), desiredState);
        speeds.omegaRadiansPerSecond = 0.0;
        _systemIO.desiredChassisSpeeds = speeds;
    }

    public double getYaw() {
        return _systemIO.heading.getRadians();
    }
    public double getPitch() {
        return _systemIO.pitch;
    }

  
    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        return _systemIO.heading;
    }
    public void zeroGyroscope() {
        _yawOffset = _imu.getYaw().getValue();
        readIMU();
        resetRobotPose(_poseEstimator.getEstimatedPosition());
    }

    public void setGyroscopeAngle(Rotation2d rotation) {
        _yawOffset = _imu.getYaw().getValue() + rotation.getDegrees();
        readIMU();
    }
    public void readIMU() {
        _systemIO.heading = Rotation2d.fromDegrees((_imu.getYaw().getValue() - _yawOffset));
        _systemIO.pitch = Units.degreesToRadians(_imu.getPitch().getValue());
    }


    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(TRAJECTORY_FOLLOWING.maxDriveVelocity, TRAJECTORY_FOLLOWING.maxDriveAcceleration)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, TRAJECTORY_FOLLOWING.maxDriveAcceleration);
    }

    public SwerveDriveKinematics getKinematics(){
        return _kinematics;
    }

    public void updateOdometry() {
        _odometry.update(
                _isRedAlliance ? getHeading().minus(new Rotation2d(Math.PI)) : getHeading(),
                new SwerveModulePosition[] {
                    _modules[NORTH_WEST_IDX].getModulePosition(),
                    _modules[SOUTH_WEST_IDX].getModulePosition(),
                    _modules[SOUTH_EAST_IDX].getModulePosition(),
                    _modules[NORTH_EAST_IDX].getModulePosition()
                });
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return _poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset position and gyroOffset of odometry
     *
     * @param position is a Pose2d (Translation2d, Rotation2d)
     *     <p>Translation2d resets odometry (X,Y) coordinates
     *     <p>Rotation2d - gyroAngle = gyroOffset
     *     <p>If Rotation2d <> gyroAngle, then robot heading will no longer equal IMU heading.
     */
    public void resetRobotPose(Pose2d position) {
        for (int module = 0; module < _modules.length; module++) {
            _modules[module].resetEncoders();
        }
//        setGyroscopeAngle(position.getRotation());
        error(" Rotation: " + getHeading().toString());

//        Rotation2d _rotation = _isRedAlliance ? getHeading().minus(new Rotation2d(Math.PI)) : getHeading();
//        Pose2d _reset = new Pose2d(_translation, _rotation);
        _poseEstimator.resetPosition(
                getHeading(),
                new SwerveModulePosition[] {
                    _modules[NORTH_WEST_IDX].getModulePosition(),
                    _modules[SOUTH_WEST_IDX].getModulePosition(),
                    _modules[SOUTH_EAST_IDX].getModulePosition(),
                    _modules[NORTH_EAST_IDX].getModulePosition()
                },
                position);
        error("Reset robot position: " + position.toString());
    }

    public TrackedObjectInfo getClosestCone() {
        TrackedObjectInfo closest = null;
        double minDistance = Double.MAX_VALUE;
        ArrayList<TrackedObjectInfo> _objectCopy = _visionProcessor.getTrackedObjects();
        if (_objectCopy.size() > 0) {
            for (TrackedObjectInfo info : _objectCopy) {
                if (info.getElement() == TrackedObjectInfo.GameElement.CONE) {
                    double distance = info.getDistance();
                    if (distance < minDistance) {
                        minDistance = distance;
                        closest = info;
                    }
                }
            }
        }
        return closest;
    }

    public TrackedObjectInfo getClosestCube() {
        TrackedObjectInfo closest = null;
        double minDistance = Double.MAX_VALUE;
        ArrayList<TrackedObjectInfo> _objectCopy = _visionProcessor.getTrackedObjects();
        if (_objectCopy.size() > 0) {
            for (TrackedObjectInfo info : _objectCopy) {
                if (info.getElement() == TrackedObjectInfo.GameElement.CUBE) {
                    double distance = info.getDistance();
                    if (distance < minDistance) {
                        minDistance = distance;
                        closest = info;
                    }
                }
            }
        }
        return closest;
    }

    public boolean isConeDetected() {
        TrackedObjectInfo obj = getClosestCone();
        return obj != null;
    }

    public boolean isCubeDetected() {
        TrackedObjectInfo obj = getClosestCube();
        return obj != null;
    }

    public void moduleMetrics() {
        for (var module : _modules) {
            module.updateDashboard();
        }
    }

    public double getDistanceToGoal() {
        return _poseEstimator
                .getEstimatedPosition()
                .getTranslation()
                .getDistance(_hoverGoal.getTranslation());
    }

    public boolean isTopSpeed() {
        return Math.abs(_modules[0].getWheelVelocity()) >= (Constants.DriveTrain.MAX_MPS - 0.2);
    }

    @Override
    public void updateDashboard() {
        metric("Swerve State", _controlState.name());
        metric("Current Heading", getHeading().getRadians());
        metric("Heading Controller Target", _headingController.getTargetHeading().getRadians());
        metric("Heading State", _headingController.getHeadingState().name());
        metric("Rotation State", getYaw());
        metric("Pitch Angle", getPitch());
        metric("Estimated X", _poseEstimator.getEstimatedPosition().getX());
        metric("Estimated Y", _poseEstimator.getEstimatedPosition().getY());
        metric(
                "Distance to goal node",
                _poseEstimator
                        .getEstimatedPosition()
                        .getTranslation()
                        .getDistance(_hoverGoal.getTranslation()));
        SmartDashboard.putData(_field);
        moduleMetrics();
    }

    public enum ControlState {
        NEUTRAL(0),
        MANUAL(1),
        POSITION(2),
        ROTATION(3),
        TRAJECTORY(4);
        private final int _value;

        ControlState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public enum Mode {
        NORMAL(0),
        SLOW(1),
        VISION(2);

        private final int _value;

        Mode(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }


    public void setMode(Mode mode) {
        _mode = mode;
    }

    public Mode getMode() {
        return _mode;
    }

    public boolean isRedAlliance() {
        return _isRedAlliance;
    }

    public Pose2d getHoverGoal() {
        return _hoverGoal;
    }

    public void setHoverGoal(Pose2d pose) {
        _hoverGoal = pose;
    }
}
