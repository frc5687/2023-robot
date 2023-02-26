/* Team 5687 (C)2020-2022 */
package org.frc5687.chargedup.subsystems;

import static org.frc5687.chargedup.Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS;
import static org.frc5687.chargedup.Constants.DriveTrain.*;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.*;
import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.lib.math.Vector2d;
import org.frc5687.lib.swerve.SwerveSetpoint;
import org.frc5687.lib.swerve.SwerveSetpointGenerator;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frc5687.lib.vision.TrackedObjectInfo;
import org.frc5687.lib.vision.VisionProcessor;

public class DriveTrain extends OutliersSubsystem {
    // Order we define swerve modules in kinematics
    // NB: must be same order as we pass to SwerveDriveKinematics
    private static final int NORTH_WEST_IDX = 0;
    private static final int SOUTH_WEST_IDX = 1;
    private static final int SOUTH_EAST_IDX = 2;
    private static final int NORTH_EAST_IDX = 3;
    private final DiffSwerveModule[] _modules;
    private final SwerveDriveKinematics _kinematics;
    private final SwerveDriveOdometry _odometry;
    private ControlState _controlState;
    private boolean _fieldRelative;
    private final Pigeon2 _imu;
    private final HolonomicDriveController _poseController;

    private boolean _slowMode =  false;

    private final SwerveHeadingController _headingController;

    // Setpoint generator for swerve.
    private final SwerveSetpointGenerator _swerveSetpointGenerator;
    private KinematicLimits _kinematicLimits = KINEMATIC_LIMITS;

    private final BaseStatusSignalValue[] _moduleSignals;
    private final SystemIO _systemIO;
    private double _yawOffset;
    private final VisionProcessor _visionProcessor;
  //  private final PhotonProcessor _photonProcessor;

    private final SwerveDrivePoseEstimator _poseEstimator;
    private final Field2d _field;

    public DriveTrain(
            OutliersContainer container,
            VisionProcessor processor,
            PhotonProcessor photonProcessor,
            Pigeon2 imu) {
        super(container);
        _visionProcessor = processor;
    //    _photonProcessor = photonProcessor;
        _imu = imu;
        _systemIO = new SystemIO();

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

        // NB: it matters which order these are defined
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

        // This should set the Pigeon to 0.
        _imu.getYaw().setUpdateFrequency(200);
        _imu.getPitch().setUpdateFrequency(200);
        _yawOffset = _imu.getYaw().getValue();
        readIMU();

        _controlState = ControlState.NEUTRAL;
        _fieldRelative = true;

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
                        VecBuilder.fill(0.04, 0.04, Units.degreesToRadians(1)),
                        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        _swerveSetpointGenerator =
                new SwerveSetpointGenerator(
                        _kinematics,
                        new Translation2d[] {
                            _modules[NORTH_WEST_IDX].getModuleLocation(),
                            _modules[SOUTH_WEST_IDX].getModuleLocation(),
                            _modules[SOUTH_EAST_IDX].getModuleLocation(),
                            _modules[NORTH_EAST_IDX].getModuleLocation()
                        });

        _headingController = new SwerveHeadingController(Constants.UPDATE_PERIOD);

        _moduleSignals = new BaseStatusSignalValue[NUM_MODULES * 4];
        for (int i = 0; i < NUM_MODULES; ++i) {
            var signals = _modules[i].getSignals();
            _moduleSignals[(i * 4)] = signals[0];
            _moduleSignals[(i * 4) + 1] = signals[1];
            _moduleSignals[(i * 4) + 2] = signals[2];
            _moduleSignals[(i * 4) + 3] = signals[3];
        }
        _field = new Field2d();
        readModules();
        setSetpointFromMeasuredModules();
    }

    public static class SystemIO {
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measuredStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
                };

        Rotation2d heading = new Rotation2d(0.0);
        double pitch = 0.0;
        // outputs
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[4]);
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

    // use for modules as controller is running at 200Hz.
    public void modulePeriodic() {
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
        //        _poseEstimator.update(
        //                _imu.getRotation2d().minus(new Rotation2d(_yawOffset)),
        //                new SwerveModulePosition[] {
        //                        _modules[NORTH_WEST_IDX].getModulePosition(),
        //                        _modules[SOUTH_WEST_IDX].getModulePosition(),
        //                        _modules[SOUTH_EAST_IDX].getModulePosition(),
        //                        _modules[NORTH_EAST_IDX].getModulePosition()
        //                }
        //        );
        //        Optional<EstimatedRobotPose> northCameraResult =
        //
        // _photonProcessor.getNorthCameraEstimatedGlobalPose(_poseEstimator.getEstimatedPosition());
        //        Optional<EstimatedRobotPose> southWestCameraResult =
        //
        // _photonProcessor.getSouthWestCameraEstimatedGlobalPose(_poseEstimator.getEstimatedPosition());
        //        Optional<EstimatedRobotPose> southEastCameraResult =
        //
        // _photonProcessor.getSouthEastCameraEstimatedGlobalPose(_poseEstimator.getEstimatedPosition());
        //
        //        if (northCameraResult.isPresent()) {
        //            EstimatedRobotPose camNorthPose = northCameraResult.get();
        //            _poseEstimator.addVisionMeasurement(
        //                    camNorthPose.estimatedPose.toPose2d(), camNorthPose.timestampSeconds
        //            );
        //        }
        //        if (southWestCameraResult.isPresent()) {
        //            EstimatedRobotPose camSW = southWestCameraResult.get();
        //            _poseEstimator.addVisionMeasurement(
        //                    camSW.estimatedPose.toPose2d(), camSW.timestampSeconds
        //            );
        //        }
        //        if (southEastCameraResult.isPresent()) {
        //            EstimatedRobotPose camSE = southEastCameraResult.get();
        //            _poseEstimator.addVisionMeasurement(
        //                    camSE.estimatedPose.toPose2d(), camSE.timestampSeconds
        //            );
        //        }
        //        _field.setRobotPose(_poseEstimator.getEstimatedPosition());
    }

    public void startModules() {
        for (DiffSwerveModule diffSwerveModule : _modules) {
            diffSwerveModule.start();
        }
    }

    public void readModules() {
        for (int module = 0; module < _modules.length; module++) {
            _systemIO.measuredStates[module] = _modules[module].getState();
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

    public ControlState getControlState() {
        return _controlState;
    }

    public ChassisSpeeds getDesiredChassisSpeeds() {
        return _systemIO.desiredChassisSpeeds;
    }

    public SwerveSetpoint getSetpoint() {
        return _systemIO.setpoint;
    }

    public void updateDesiredStates() {
        //        Pose2d robotPoseVel = new Pose2d(
        //                _systemIO.desiredChassisSpeeds.vxMetersPerSecond * Constants.CONTROL_PERIOD,
        //                _systemIO.desiredChassisSpeeds.vyMetersPerSecond * Constants.CONTROL_PERIOD,
        //                Rotation2d.fromRadians(_systemIO.desiredChassisSpeeds.omegaRadiansPerSecond *
        // Constants.CONTROL_PERIOD)
        //        );
        //
        //        Twist2d twistVel = new Pose2d().log(robotPoseVel);
        //        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(
        //                twistVel.dx / Constants.CONTROL_PERIOD,
        //                twistVel.dy / Constants.CONTROL_PERIOD,
        //                twistVel.dtheta / Constants.CONTROL_PERIOD
        //        );
        _systemIO.setpoint =
                _swerveSetpointGenerator.generateSetpoint(
                        _kinematicLimits,
                        _systemIO.setpoint,
                        _systemIO.desiredChassisSpeeds,
                        //                updatedChassisSpeeds,
                        0.02);
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        _systemIO.desiredChassisSpeeds = chassisSpeeds;
    }

    public void setVelocityPose(Pose2d pose) {
        ChassisSpeeds speeds =
                _poseController.calculate(
                        _poseEstimator.getEstimatedPosition(), pose, 0.0, _imu.getRotation2d());
        speeds.omegaRadiansPerSecond = 0.0;
        _systemIO.desiredChassisSpeeds = speeds;
    }

    public void updateSwerve(Vector2d translationVector, double rotationalInput) {
        SwerveModuleState[] swerveModuleStates =
                _kinematics.toSwerveModuleStates(
                        _fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translationVector.x(), translationVector.y(), rotationalInput, getHeading())
                                : new ChassisSpeeds(translationVector.x(), translationVector.y(), rotationalInput));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(swerveModuleStates);
    }

    public void updateSwerve(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds adjustedSpeeds = _poseController.calculate(getOdometryPose(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(moduleStates);
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
    }

    public void readIMU() {
        _systemIO.heading = Rotation2d.fromDegrees((_imu.getYaw().getValue() - _yawOffset));
        _systemIO.pitch = Units.degreesToRadians(_imu.getPitch().getValue());
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(Constants.DriveTrain.MAX_AUTO_MPS, Constants.DriveTrain.MAX_MPSS)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, Constants.DriveTrain.MAX_AUTO_MPS);
    }

    public void setKinematicLimits(KinematicLimits limits) {
        if (limits != KINEMATIC_LIMITS) {
            _kinematicLimits = limits;
        }
    }

    public void updateOdometry() {
        _odometry.update(
                getHeading(),
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
    public void resetOdometry(Pose2d position) {
        for (int module = 0; module < _modules.length; module++) {
            _modules[module].resetEncoders();
        }
        Translation2d _translation = position.getTranslation();
        Rotation2d _rotation = getHeading();
        Pose2d _reset = new Pose2d(_translation, _rotation);
        _odometry.resetPosition(
                getHeading(),
                new SwerveModulePosition[] {
                    _modules[NORTH_WEST_IDX].getModulePosition(),
                    _modules[SOUTH_WEST_IDX].getModulePosition(),
                    _modules[SOUTH_EAST_IDX].getModulePosition(),
                    _modules[NORTH_EAST_IDX].getModulePosition()
                },
                _reset);
    }

    public void setFieldRelative(boolean relative) {
        _fieldRelative = relative;
    }

    public boolean isFieldRelative() {
        return _fieldRelative;
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

    @Override
    public void updateDashboard() {
        metric("Swerve State", _controlState.name());
        metric("Odometry Pose", getOdometryPose().toString());
        metric("Current Heading", getHeading().getRadians());
        metric("Heading Controller Target", _headingController.getTargetHeading().getRadians());
        metric("Heading State", _headingController.getHeadingState().name());
        metric("Rotation State", getYaw());
        metric("Pitch Angle", getPitch());
        metric("Estimated X", _poseEstimator.getEstimatedPosition().getX());
        metric("Estimated Y", _poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putData(_field);
        //        metric("Pitch Angle Deg", Units.radiansToDegrees(getPitch()));
        moduleMetrics();
        //        metric("NW Angle", _modules[NORTH_WEST_IDX].getModuleAngle());
        //        metric("SW Angle", _modules[SOUTH_WEST_IDX].getModuleAngle());
        //        metric("SE Angle", _modules[SOUTH_EAST_IDX].getModuleAngle());
        //        metric("NE Angle", _modules[NORTH_EAST_IDX].getModuleAngle());
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

    public HeadingState getHeadingControllerState() {
        return _headingController.getHeadingState();
    }

    public void setHeadingControllerState(HeadingState state) {
        _headingController.setState(state);
    }

    public double getRotationCorrection() {
        return _headingController.getRotationCorrection(getHeading());
    }
    public void setSlowMode(boolean slow) {
        _slowMode = slow;
    }
    public boolean getSlowMode() {
        return _slowMode;
    }
}
