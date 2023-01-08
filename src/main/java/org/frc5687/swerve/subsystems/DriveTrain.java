/* Team 5687 (C)2020-2022 */
package org.frc5687.swerve.subsystems;

import static org.frc5687.swerve.Constants.DifferentialSwerveModule.*;
import static org.frc5687.swerve.Constants.DriveTrain.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import org.frc5687.swerve.Constants;
import org.frc5687.swerve.OI;
import org.frc5687.swerve.RobotMap;
import org.frc5687.swerve.util.OutliersContainer;

public class DriveTrain extends OutliersSubsystem {

    private static final int NORTH_WEST = 0;
    private static final int SOUTH_WEST = 1;
    private static final int SOUTH_EAST = 2;
    private static final int NORTH_EAST = 3;
    private DiffSwerveModule _northEast;
    private DiffSwerveModule _northWest;
    private DiffSwerveModule _southEast;
    private DiffSwerveModule _southWest;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    private double _PIDAngle;

    private AHRS _imu;
    private OI _oi;

    private HolonomicDriveController _controller;
    private ProfiledPIDController _angleController;

    public DriveTrain(OutliersContainer container, OI oi, AHRS imu) {
        super(container);
        _oi = oi;
        _imu = imu;
        _northWest =
                new DiffSwerveModule(
                        Constants.DriveTrain.NORTH_WEST,
                        RobotMap.CAN.TALONFX.NORTH_WEST_OUTER,
                        RobotMap.CAN.TALONFX.NORTH_WEST_INNER,
                        RobotMap.DIO.NORTH_WEST,
                        Constants.DriveTrain.NORTH_WEST_OFFSET,
                        Constants.DriveTrain.NORTH_WEST_ENCODER_INVERTED);
        _southWest =
                new DiffSwerveModule(
                        Constants.DriveTrain.SOUTH_WEST,
                        RobotMap.CAN.TALONFX.SOUTH_WEST_OUTER,
                        RobotMap.CAN.TALONFX.SOUTH_WEST_INNER,
                        RobotMap.DIO.SOUTH_WEST,
                        Constants.DriveTrain.SOUTH_WEST_OFFSET,
                        Constants.DriveTrain.SOUTH_WEST_ENCODER_INVERTED);
        _southEast =
                new DiffSwerveModule(
                        Constants.DriveTrain.SOUTH_EAST,
                        RobotMap.CAN.TALONFX.SOUTH_EAST_INNER,
                        RobotMap.CAN.TALONFX.SOUTH_EAST_OUTER,
                        RobotMap.DIO.SOUTH_EAST,
                        Constants.DriveTrain.SOUTH_EAST_OFFSET,
                        Constants.DriveTrain.SOUTH_EAST_ENCODER_INVERTED);
        _northEast =
                new DiffSwerveModule(
                        Constants.DriveTrain.NORTH_EAST,
                        RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                        RobotMap.CAN.TALONFX.NORTH_EAST_OUTER,
                        RobotMap.DIO.NORTH_EAST,
                        Constants.DriveTrain.NORTH_EAST_OFFSET,
                        Constants.DriveTrain.NORTH_EAST_ENCODER_INVERTED);
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

        _controller =
                new HolonomicDriveController(
                        new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                        new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                        new ProfiledPIDController(
                                Constants.DriveTrain.kP,
                                Constants.DriveTrain.kI,
                                Constants.DriveTrain.kD,
                                new TrapezoidProfile.Constraints(
                                        Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));
        _angleController =
                new ProfiledPIDController(
                        Constants.DriveTrain.ANGLE_kP,
                        Constants.DriveTrain.ANGLE_kI,
                        Constants.DriveTrain.ANGLE_kD,
                        new TrapezoidProfile.Constraints(
                                Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL));
        _angleController.enableContinuousInput(-Math.PI / 2.0, Math.PI / 2.0);
    }

    // use for modules as controller is running at 200Hz.
    public void controllerPeriodic() {
        _northEast.periodic();
        _northWest.periodic();
        _southEast.periodic();
        _southWest.periodic();
    }

    @Override
    public void periodic() {
        _odometry.update(
                getHeading(),
                new SwerveModulePosition[] {
                _northWest.getModulePosition(),
                _northEast.getModulePosition(),
                _southWest.getModulePosition(),
                _southEast.getModulePosition() }
                );
    }

    @Override
    public void updateDashboard() {
        metric("SE/Encoder Angle", _southEast.getModuleAngle());
        metric("SW/Encoder Angle", _southWest.getModuleAngle());
        metric("NW/Encoder Angle", _northWest.getModuleAngle());
        metric("NE/Encoder Angle", _northEast.getModuleAngle());

        metric("SE/Predicted Angle", _southEast.getPredictedAzimuthAngle());

        metric("SE/Encoder Azimuth Vel", _southEast.getAzimuthAngularVelocity());
        metric("SE/Predicted Azimuth Vel", _southEast.getPredictedAzimuthAngularVelocity());

        metric("SE/Encoder Wheel Vel", _southEast.getWheelVelocity());
        metric("SE/Predicted Wheel Vel", _southEast.getPredictedWheelVelocity());

        metric("Odometry Pose", getOdometryPose().toString());
    }

    public void setNorthEastModuleState(SwerveModuleState state) {
        _northEast.setIdealState(state);
    }

    public void setNorthWestModuleState(SwerveModuleState state) {
        _northWest.setIdealState(state);
    }

    public void setSouthWestModuleState(SwerveModuleState state) {
        _southWest.setIdealState(state);
    }

    public void setSouthEastModuleState(SwerveModuleState state) {
        _southEast.setIdealState(state);
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

    /**
     * Method to set correct module speeds and angle based on wanted vx, vy, omega
     *
     * @param vx velocity in x direction
     * @param vy velocity in y direction
     * @param omega angular velocity (rotating speed)
     * @param fieldRelative forward is always forward no mater orientation of robot.
     */
    public void drive(double vx, double vy, double omega, boolean fieldRelative) {
        if (Math.abs(vx) < DEADBAND && Math.abs(vy) < DEADBAND && Math.abs(omega) < DEADBAND) {
            setNorthEastModuleState(
                    new SwerveModuleState(0, new Rotation2d(_northEast.getModuleAngle())));
            setNorthWestModuleState(
                    new SwerveModuleState(0, new Rotation2d(_northWest.getModuleAngle())));
            setSouthEastModuleState(
                    new SwerveModuleState(0, new Rotation2d(_southEast.getModuleAngle())));
            setSouthWestModuleState(
                    new SwerveModuleState(0, new Rotation2d(_southWest.getModuleAngle())));
            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else if (Math.abs(omega) > 0) {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                            fieldRelative
                                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                            vx, vy, omega, getHeading())
                                    : new ChassisSpeeds(vx, vy, omega));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
            setNorthWestModuleState(swerveModuleStates[NORTH_WEST]);
            setSouthWestModuleState(swerveModuleStates[SOUTH_WEST]);
            setSouthEastModuleState(swerveModuleStates[SOUTH_EAST]);
            setNorthEastModuleState(swerveModuleStates[NORTH_EAST]);
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
            setNorthWestModuleState(swerveModuleStates[NORTH_WEST]);
            setSouthWestModuleState(swerveModuleStates[SOUTH_WEST]);
            setSouthEastModuleState(swerveModuleStates[SOUTH_EAST]);
            setNorthEastModuleState(swerveModuleStates[NORTH_EAST]);
        }
    }

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, MAX_MPS);
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(MAX_MPS, MAX_MPSS)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public void trajectoryFollower(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds adjustedSpeeds =
                _controller.calculate(_odometry.getPoseMeters(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_SPEED_MPS);
        setNorthWestModuleState(moduleStates[NORTH_WEST]);
        setSouthWestModuleState(moduleStates[SOUTH_WEST]);
        setSouthEastModuleState(moduleStates[SOUTH_EAST]);
        setNorthEastModuleState(moduleStates[NORTH_EAST]);
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
    }

    public void startModules() {
        _northEast.start();
        _northWest.start();
        _southWest.start();
        _southEast.start();
    }
}
