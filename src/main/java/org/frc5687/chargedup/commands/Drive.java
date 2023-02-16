/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.lib.control.HeadingController;
import org.frc5687.chargedup.util.Helpers;
import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.math.Vector2d;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
//    private final HeadingController _headingController;
    private final SwerveHeadingController _headingController;
    private final OI _oi;
    private boolean _lockHeading;
    private int segmentationArray[] = new int[((int)360/5)];

    public Drive(DriveTrain driveTrain, OI oi) {
        _lockHeading = false;
        _driveTrain = driveTrain;
        _headingController = new SwerveHeadingController(Constants.UPDATE_PERIOD);
//        _headingController = new HeadingController(
//                new TrapezoidProfile.Constraints(
//                        Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
//                        Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL
//                )
//        );
        _oi = oi;

        for (int i = 0; i < segmentationArray.length; i++){
            double angle = 360 / segmentationArray.length;
            segmentationArray[i] = (int)angle * i;
        }
        addRequirements(_driveTrain);
        //        logMetrics("vx","vy");
        //        enableMetrics();

    }

    @Override
    public void initialize() {
        _driveTrain.startModules();
        _driveTrain.setFieldRelative(true);
//        _headingController.setGoal(_driveTrain.getHeading().getRadians());
        _headingController.setMaintainHeading(_driveTrain.getHeading());
//        _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
    }

    @Override
    public void execute() {
        if (_oi.zeroIMU()) {
            _driveTrain.zeroGyroscope();
            _headingController.setState(SwerveHeadingController.HeadingState.OFF);
            _headingController.getRotationCorrection(_driveTrain.getHeading());
        }
        //  driveX and driveY are swapped due to coordinate system that WPILib uses.
        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(_oi.getDriveY(), _oi.getDriveX(), segmentationArray);
        double vx;
        double vy;
        double rot = _oi.getRotationX();
        rot = Math.signum(rot) * rot * rot;
        //  driveX and driveY are swapped due to coordinate system that WPILib uses
        if (_oi.getSlowMode()) {
            vx = vec.x() * Constants.DriveTrain.SLOW_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
            vy = vec.y() * Constants.DriveTrain.SLOW_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
            rot = -rot * Constants.DriveTrain.SLOW_ANG_VEL * Constants.DriveTrain.SCALED_ROTATION_INPUT; //negative added to flip rotation in slowmode, driver preference
        } else {
            vx = vec.x() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
            vy = vec.y() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
            rot = rot * Constants.DriveTrain.MAX_ANG_VEL * Constants.DriveTrain.SCALED_ROTATION_INPUT;
//        double vx = _oi.getDriveY() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
//        double vy = _oi.getDriveX() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
        }
        

        // 0.01 is the tolerance to start heading controller.
//        if (Math.abs(rot) < 0.01) {
//            if (_oi.setHeadingNorth()) {
//                _headingController.setSnapHeading(new Rotation2d(0.0));
//            } else if (_oi.setHeadingEast()) {
//                _headingController.setSnapHeading(new Rotation2d(-Math.PI / 2.0));
//            } else if (_oi.setHeadingSouth()) {
//                _headingController.setSnapHeading(new Rotation2d(Math.PI));
//            } else if (_oi.setHeadingWest()) {
//                _headingController.setSnapHeading(new Rotation2d(Math.PI / 2.0));
//            } else if (_headingController.getHeadingState() == SwerveHeadingController.HeadingState.SNAP) {
//                _headingController.setState(SwerveHeadingController.HeadingState.MAINTAIN);
//                _headingController.setMaintainHeading(_driveTrain.getHeading());
////                _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.MAINTAIN);
////                _headingController.setGoal(_headingController.getGoal());
//            } else if (_headingController.getHeadingControllerState() == HeadingController.HeadingControllerState.OFF) {
//                _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.MAINTAIN);
//            }
//        } else {
//            _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.OFF);
//            _headingController.reset();
//            _headingController.setGoal(_driveTrain.getHeading().getRadians());
//        }
//        if (_oi.setHeadingNorth()) {
//            _headingController.setSnapHeading(new Rotation2d(0));
//            _headingController.setState(SwerveHeadingController.HeadingState.SNAP);
//            _lockHeading = true;
////            _headingController.setSnapHeading(new Rotation2d(0));
//        } else if (){
//            _lockHeading = false;
//        }

        if (rot == 0) {
            if (!_lockHeading) {
                _headingController.temporaryDisable();
            }
            _lockHeading = true;
        } else {
            _headingController.disable();
            _lockHeading = false;
        }
        double controllerPower = _headingController.getRotationCorrection(_driveTrain.getHeading());
        metric("Rot", (rot));
        metric("Heading",_driveTrain.getHeading().getRadians());
        metric("vx", vx);
        metric("vy", vy);
//        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//                vx,
//                vy,
//                rot,
//                _driveTrain.getHeading()
//        );
        _driveTrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                vx,
                vy,
                rot +controllerPower,
                _driveTrain.getHeading()
        ));
//        _driveTrain.drive(vx, vy, rot);
//        metric("ChassisSpeeds", speeds.toString());
//        _driveTrain.updateSwerve(new Vector2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond);
//        _driveTrain.updateSwerve(new Vector2d(vx, vy), rot);
//        _driveTrain.updateSwerve(new Vector2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
