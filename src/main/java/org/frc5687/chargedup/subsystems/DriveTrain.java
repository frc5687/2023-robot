/* Team 5687 (C)2020-2022 */
package org.frc5687.chargedup.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import org.frc5687.lib.math.Vector2d;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.*;
import org.frc5687.lib.swerve.SwerveSetpoint;
import org.frc5687.lib.swerve.SwerveSetpointGenerator;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;

import static org.frc5687.chargedup.Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS;
import static org.frc5687.chargedup.Constants.DriveTrain.*;

public class DriveTrain extends OutliersSubsystem {
    // Order we define swerve modules in kinematics
    // NB: must be same order as we pass to SwerveDriveKinematics
    private static final int NORTH_WEST_IDX = 0;
    private static final int SOUTH_WEST_IDX = 1;
    private static final int SOUTH_EAST_IDX = 2;
    private static final int NORTH_EAST_IDX = 3;
    private final DiffSwerveModuleCurrent[] _modules;
    private final SwerveDriveKinematics _kinematics;
    private final SwerveDriveOdometry _odometry;
    private ControlState _controlState;
    private boolean _fieldRelative;
    private final Pigeon2 _imu;
    private final OI _oi;
    private final HolonomicDriveController _poseController;
    private ProfiledPIDController _angleController;

    // Setpoint generator for swerve.
    private final SwerveSetpointGenerator _swerveSetpointGenerator;
    private KinematicLimits _kinematicLimits = KINEMATIC_LIMITS;

    private final SystemIO _systemIO;
    private double _yawOffset;
    private double _PIDAngle;

    public DriveTrain(OutliersContainer container, OI oi, Pigeon2 imu) {
        super(container);
        _imu = imu;
        _oi = oi;
        _systemIO = new SystemIO();
        _PIDAngle = getHeading().getRadians();

        _modules = new DiffSwerveModuleCurrent[4];

        _modules[NORTH_WEST_IDX] = new DiffSwerveModuleCurrent(
                NORTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_WEST_OUTER,
                RobotMap.CAN.TALONFX.NORTH_WEST_INNER,
                RobotMap.DIO.ENCODER_NW
        );
        _modules[SOUTH_WEST_IDX] = new DiffSwerveModuleCurrent(
                SOUTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_WEST_OUTER,
                RobotMap.CAN.TALONFX.SOUTH_WEST_INNER,
                RobotMap.DIO.ENCODER_SW
        );
        _modules[SOUTH_EAST_IDX] = new DiffSwerveModuleCurrent(
                SOUTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_EAST_INNER,
                RobotMap.CAN.TALONFX.SOUTH_EAST_OUTER,
                RobotMap.DIO.ENCODER_SE
        );
        _modules[NORTH_EAST_IDX] = new DiffSwerveModuleCurrent(
                NORTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                RobotMap.CAN.TALONFX.NORTH_EAST_OUTER,
                RobotMap.DIO.ENCODER_NE
        );


        // NB: it matters which order these are defined
        _kinematics =
                new SwerveDriveKinematics(
                        _modules[NORTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_EAST_IDX].getModuleLocation(),
                        _modules[NORTH_EAST_IDX].getModuleLocation()
                );
        _odometry = new SwerveDriveOdometry(
                _kinematics,
                getHeading(),
                new SwerveModulePosition[]{
                        _modules[NORTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_EAST_IDX].getModulePosition(),
                        _modules[NORTH_EAST_IDX].getModulePosition()
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
                                MAINTAIN_kP,
                                MAINTAIN_kI,
                                MAINTAIN_kP,
                                new TrapezoidProfile.Constraints(
                                        Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                        Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL))
                );

        _angleController = new ProfiledPIDController(
                Constants.DriveTrain.kP,
                Constants.DriveTrain.kI,
                Constants.DriveTrain.kD,
                new TrapezoidProfile.Constraints(Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)
        );
        // This should set the Pigeon to 0.
        _yawOffset = _imu.getYaw();
        readIMU();

        readModules();
        setSetpointFromMeasuredModules();

        _imu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 5);

        _controlState = ControlState.NEUTRAL;
        _fieldRelative = true;

        _swerveSetpointGenerator = new SwerveSetpointGenerator(
                _kinematics,
                new Translation2d[] {
                        _modules[NORTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_EAST_IDX].getModuleLocation(),
                        _modules[NORTH_EAST_IDX].getModuleLocation()
                }
        );

    }
    public static class SystemIO {
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measuredStates = new SwerveModuleState[] {
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
//        for (DiffSwerveModuleCurrent diffSwerveModule : _modules) {
//            diffSwerveModule.periodic();
//        }
        _modules[0].periodic();
    }
    public void drive(double vx, double vy, double omega) {
        if (Math.abs(vx) < TRANSLATION_DEADBAND && Math.abs(vy) < TRANSLATION_DEADBAND && Math.abs(omega) < ROTATION_DEADBAND) {
            for (DiffSwerveModuleCurrent diffSwerveModule : _modules) {
                diffSwerveModule.setIdealState(new SwerveModuleState(0.0, new Rotation2d(diffSwerveModule.getModuleAngle())));
            }

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
    @Override
    public void controlPeriodic(double timestamp) {
        // modulePeriodic();
        // read sensors and modules so that they are cached for this loop
        readIMU();
        readModules();
//        switch (_controlState) {
//            case TRAJECTORY:
//                break;
//            case NEUTRAL:
//            case MANUAL:
//            case ROTATION:
//            default:
//                break;
//        }
        updateDesiredStates();
        setModuleStates(_systemIO.setpoint.moduleStates);
    }

    @Override
    public void dataPeriodic(double timestamp) {
        updateOdometry();
    }

    public void startModules() {
        for (DiffSwerveModuleCurrent diffSwerveModule : _modules) {
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
        System.arraycopy(_systemIO.measuredStates, 0, _systemIO.setpoint.moduleStates, 0, _modules.length);
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
        Pose2d robotPoseVel = new Pose2d(
                _systemIO.desiredChassisSpeeds.vxMetersPerSecond * Constants.CONTROL_PERIOD,
                _systemIO.desiredChassisSpeeds.vyMetersPerSecond * Constants.CONTROL_PERIOD,
                Rotation2d.fromRadians(_systemIO.desiredChassisSpeeds.omegaRadiansPerSecond * Constants.CONTROL_PERIOD)
        );

        Twist2d twistVel = new Pose2d().log(robotPoseVel);
        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(
                twistVel.dx / Constants.CONTROL_PERIOD,
                twistVel.dy / Constants.CONTROL_PERIOD,
                twistVel.dtheta / Constants.CONTROL_PERIOD
        );
        _systemIO.setpoint = _swerveSetpointGenerator.generateSetpoint(
                _kinematicLimits,
                _systemIO.setpoint,
                // _systemIO.desiredChassisSpeeds,
                updatedChassisSpeeds,
                Constants.CONTROL_PERIOD
        );
        metric("Setpoint Swerve", _systemIO.setpoint.chassisSpeeds.toString());

    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        _systemIO.desiredChassisSpeeds = chassisSpeeds;
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

    public double getYaw() {
        return _systemIO.heading.getRadians();
    }

    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        return _systemIO.heading;
    }

    public void zeroGyroscope() {
        _yawOffset = _imu.getYaw();
        readIMU();
    }

    public void readIMU() {
        _systemIO.heading = Rotation2d.fromDegrees(_imu.getYaw() - _yawOffset);
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

    public void updateOdometry() {
        _odometry.update(
                getHeading(),
                new SwerveModulePosition[]{
                        _modules[NORTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_EAST_IDX].getModulePosition(),
                        _modules[NORTH_EAST_IDX].getModulePosition()
                }
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
        for (int module = 0; module < _modules.length; module++) {
            _modules[module].resetEncoders();
        }
        Translation2d _translation = position.getTranslation();
        Rotation2d _rotation = getHeading();
        Pose2d _reset = new Pose2d(_translation, _rotation);
        _odometry.resetPosition(
                getHeading(),
                new SwerveModulePosition[]{
                        _modules[NORTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_EAST_IDX].getModulePosition(),
                        _modules[NORTH_EAST_IDX].getModulePosition()
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
        metric("Rotation State", getYaw());
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
}
