/* Team 5687 (C)2020-2022 */
package org.frc5687.chargedup.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import org.frc5687.lib.math.GeometryUtil;
import org.frc5687.lib.math.Vector2d;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.*;
import org.frc5687.lib.swerve.SwerveSetpoint;
import org.frc5687.lib.swerve.SwerveSetpointGenerator;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;

import java.util.Arrays;
import java.util.List;

import static org.frc5687.chargedup.Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS;
import static org.frc5687.chargedup.Constants.DriveTrain.*;

public class DriveTrain extends OutliersSubsystem {
    // Order we define swerve modules in kinematics
    // NB: must be same order as we pass to SwerveDriveKinematics
    private DiffSwerveModule _northWest, _southWest, _northEast, _southEast;
    private List<DiffSwerveModule> _modules;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    private ControlState _controlState;
    private boolean _fieldRelative;

    private Translation2d _clockwiseCenter;
    private Translation2d _counterClockwiseCenter;

    private AHRS _imu;
    private OI _oi;
    private HolonomicDriveController _poseController;
    private ProfiledPIDController _angleController;

    // Setpoint generator for swerve.
    private SwerveSetpointGenerator _swerveSetpointGenerator;
    private KinematicLimits _kinematicLimits = KINEMATIC_LIMITS;

    private double _PIDAngle;

    private SystemIO _systemIO;

    public DriveTrain(OutliersContainer container, OI oi, AHRS imu) {
        super(container);
        _imu = imu;
        _oi = oi;
        _systemIO = new SystemIO();

        _northWest = new DiffSwerveModule(
                NORTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_WEST_OUTER,
                RobotMap.CAN.TALONFX.NORTH_WEST_INNER,
                RobotMap.DIO.ENCODER_NW
        );
        _southWest = new DiffSwerveModule(
                SOUTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_WEST_OUTER,
                RobotMap.CAN.TALONFX.SOUTH_WEST_INNER,
                RobotMap.DIO.ENCODER_SW
        );
        _southEast = new DiffSwerveModule(
                SOUTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_EAST_INNER,
                RobotMap.CAN.TALONFX.SOUTH_EAST_OUTER,
                RobotMap.DIO.ENCODER_SE
        );
        _northEast = new DiffSwerveModule(
                NORTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                RobotMap.CAN.TALONFX.NORTH_EAST_OUTER,
                RobotMap.DIO.ENCODER_NE
        );

        _modules = Arrays.asList(_northWest, _southWest, _southEast, _northEast);

        // NB: it matters which order these are defined
        _kinematics =
                new SwerveDriveKinematics(
                        _northWest.getModuleLocation(),
                        _southWest.getModuleLocation(),
                        _southEast.getModuleLocation(),
                        _northEast.getModuleLocation()
                );
        _odometry = new SwerveDriveOdometry(
                _kinematics,
                getHeading(),
                new SwerveModulePosition[]{
                        _northWest.getModulePosition(),
                        _southWest.getModulePosition(),
                        _southEast.getModulePosition(),
                        _northEast.getModulePosition()
                },
                new Pose2d(0, 0, getHeading())
        );

        _poseController =
                new HolonomicDriveController(
                        new PIDController(
                                Constants.DriveTrain.kP,
                                Constants.DriveTrain.kI,
                                Constants.DriveTrain.kD),
                        new PIDController(
                                Constants.DriveTrain.kP,
                                Constants.DriveTrain.kI,
                                Constants.DriveTrain.kD),
                        new ProfiledPIDController(
                                STABILIZATION_kP,
                                STABILIZATION_kI,
                                STABILIZATION_kD,
                                new TrapezoidProfile.Constraints(
                                        Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                        Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));

        _angleController = new ProfiledPIDController(
            Constants.DriveTrain.kP,
            Constants.DriveTrain.kI,
            Constants.DriveTrain.kD,
            new TrapezoidProfile.Constraints(Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)
        );

        _clockwiseCenter = new Translation2d();
        _counterClockwiseCenter = new Translation2d();

        _controlState = ControlState.NEUTRAL;
        _fieldRelative = true;

        _swerveSetpointGenerator = new SwerveSetpointGenerator(
                _kinematics,
                new Translation2d[] {
                        _modules.get(0).getModuleLocation(),
                        _modules.get(1).getModuleLocation(),
                        _modules.get(2).getModuleLocation(),
                        _modules.get(3).getModuleLocation()
                }
        );
    }

    public static class SystemIO {
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measuredStated = new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };

        Rotation2d heading = new Rotation2d(0.0);
        // outputs
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[4]);
    }

    // use for modules as controller is running at 200Hz.
    public void modulePeriodic() {
        _modules.forEach(DiffSwerveModule::periodic);
    }

    @Override
    public void controlPeriodic(double timestamp) {
        modulePeriodic();
    }

    @Override
    public void dataPeriodic(double timestamp) {
        updateOdometry();
    }

    public void startModules() {
        _modules.forEach(DiffSwerveModule::start);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int module = 0; module < _modules.size(); module++) {
            _modules.get(module).setIdealState(states[module]);
        }
    }

    public void setControlState(ControlState state) {
        _controlState = state;
    }

    public ControlState getControlState() {
        return _controlState;
    }

    /**
     * Method to set correct module speeds and angle based on wanted vx, vy, omega
     *
     * @param vx velocity in x direction
     * @param vy velocity in y direction
     * @param omega angular velocity (rotating speed)
     */
    public void drive(double vx, double vy, double omega) {
        if (Math.abs(vx) < TRANSLATION_DEADBAND && Math.abs(vy) < TRANSLATION_DEADBAND && Math.abs(omega) < ROTATION_DEADBAND) {
            _northEast.setIdealState(new SwerveModuleState(0, new Rotation2d(_northEast.getModuleAngle())));
            _northWest.setIdealState(new SwerveModuleState(0, new Rotation2d(_northWest.getModuleAngle())));
            _southEast.setIdealState(new SwerveModuleState(0, new Rotation2d(_southEast.getModuleAngle())));
            _southWest.setIdealState(new SwerveModuleState(0, new Rotation2d(_southWest.getModuleAngle())));

            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else if (Math.abs(omega) > 0) {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                           _fieldRelative
                                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    vx, vy, omega, getHeading())
                                    : new ChassisSpeeds(vx, vy, omega));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
            setModuleStates(swerveModuleStates);
            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    vx,
                                    vy,
                                    _angleController.calculate(
                                            getHeading().getRadians(), _PIDAngle),
                                    new Rotation2d(_PIDAngle)));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
            setModuleStates(swerveModuleStates);
        }
    }

    private void updateDesiredStates() {
        Pose2d robotPoseVel = new Pose2d()
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {

    }
    public void updateSwerve(Vector2d translationVector, double rotationalInput) {
        SwerveModuleState[] swerveModuleStates =
                _kinematics.toSwerveModuleStates(
                        _fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translationVector.x(),
                                        translationVector.y(),
                                        rotationalInput,
                                        getHeading())
                                : new ChassisSpeeds(
                                        translationVector.x(),
                                        translationVector.y(),
                                        rotationalInput)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(swerveModuleStates);
    }

    public void updateSwerve(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds adjustedSpeeds = _poseController.calculate(getOdometryPose(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(moduleStates);
    }

    public void updateSwerve(Pose2d pose) {
        ChassisSpeeds adjustedSpeeds =
                _poseController.calculate(
                        getOdometryPose(), pose, LINEAR_VELOCITY_REFERENCE, pose.getRotation());
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(moduleStates);
    }

    public double getYaw() {
        return _imu.getYaw();
    }

    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-getYaw());
    }

    public void resetYaw() {
        _imu.reset();
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(
                        Constants.DriveTrain.MAX_AUTO_MPS, Constants.DriveTrain.MAX_MPSS)
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

    public boolean isAtPose(Pose2d pose) {
        double diffX = getOdometryPose().getX() - pose.getX();
        double diffY = getOdometryPose().getY() - pose.getY();
        return (Math.abs(diffX) <= Constants.DriveTrain.POSITION_TOLERANCE)
                && (Math.abs(diffY) < Constants.DriveTrain.POSITION_TOLERANCE);
    }

    public void updateOdometry() {
        _odometry.update(
                getHeading(),
                new SwerveModulePosition[] {
                        _northWest.getModulePosition(),
                        _northEast.getModulePosition(),
                        _southWest.getModulePosition(),
                        _southEast.getModulePosition() }
        );
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
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
        _modules.forEach(DiffSwerveModule::resetEncoders);
        Translation2d _translation = position.getTranslation();
        Rotation2d _rotation = getHeading();
        Pose2d _reset = new Pose2d(_translation, _rotation);
        _odometry.resetPosition(
                getHeading(),
                new SwerveModulePosition[]{
                        _northWest.getModulePosition(),
                        _southWest.getModulePosition(),
                        _southEast.getModulePosition(),
                        _northEast.getModulePosition()
                },
                _reset
                );
    }

    public void setFieldRelative(boolean relative) {
        _fieldRelative = relative;
    }

    public boolean isFieldRelative() {
        return _fieldRelative;
    }

    @Override
    public void updateDashboard() {
        metric("Swerve State", _controlState.name());
        metric("Odometry Pose", getOdometryPose().toString());
        metric("Current Heading", getHeading().getRadians());
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
}
