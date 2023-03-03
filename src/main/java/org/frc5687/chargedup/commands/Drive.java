/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.DriveTrain.ControlState;
import org.frc5687.lib.control.HeadingController;
import org.frc5687.chargedup.util.Helpers;
import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.math.Vector2d;
import org.frc5687.lib.vision.TrackedObjectInfo;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final EndEffector _endEffector;
//    private final HeadingController _headingController;
    private final PIDController _yCoordinateElementController;
    private final OI _oi;
    private boolean _lockHeading;
    private int segmentationArray[] = new int[((int) 360 / 5)];

    public Drive(DriveTrain driveTrain, EndEffector endEffector, OI oi) {
        _lockHeading = false;
        _driveTrain = driveTrain;
        _endEffector = endEffector;
        _oi = oi;
        _yCoordinateElementController = new PIDController(
                2.0,
                0.0,
                0.2
        );
//        _headingController = new HeadingController(
//                new TrapezoidProfile.Constraints(
//                        Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
//                        Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL
//                )
//        );

        for (int i = 0; i < segmentationArray.length; i++) {
            double angle = 360 / segmentationArray.length;
            segmentationArray[i] = (int) angle * i;
        }
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        _driveTrain.startModules();
        _driveTrain.setFieldRelative(true);
//        _headingController.setGoal(_driveTrain.getHeading().getRadians());
        // _headingController.setMaintainHeading(_driveTrain.getHeading());
        _driveTrain.setPrevVelocity(_driveTrain.getDesiredChassisSpeeds());
        _driveTrain.determineCORForEvasion(_oi.getTranslationVector());
       _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
    }

    @Override
    public void execute() {
        if (_oi.zeroIMU()) {
            _driveTrain.zeroGyroscope();
            _driveTrain.setHeadingControllerState(SwerveHeadingController.HeadingState.OFF);
            _driveTrain.getRotationCorrection();
        }

        if ((_oi.getTranslationVector().x() - _driveTrain.getPrevVector2d().x() > Constants.DriveTrain.PREV_VECTOR_CHANGE_X || _oi.getTranslationVector().y() - _driveTrain.getPrevVector2d().y() > Constants.DriveTrain.PREV_VECTOR_CHANGE_Y) && !_oi.getTranslationVector().equals(new Vector2d(0, 0))){
            _driveTrain.setPrevVector2d(_oi.getTranslationVector().x(), _oi.getTranslationVector().y());
        }
        _driveTrain.determineCORForEvasion(_driveTrain.getPrevVector2d());

        if (_oi.setCORLeft()){
            _driveTrain.setCenterOfRotation(_driveTrain.getCounterClockwiseCOR());
        }
        if (_oi.setCORRight()){
            _driveTrain.setCenterOfRotation(_driveTrain.getClockwiseCOR());
        }
        //  driveX and driveY are swapped due to coordinate system that WPILib uses.
        Vector2d vec =
                Helpers.axisToSegmentedUnitCircleRadians(
                        _oi.getDriveY(), _oi.getDriveX(), segmentationArray);
        double vx;
        double vy;
        double rot = _oi.getRotationX();
        rot = Math.signum(rot) * rot * rot;
        //  driveX and driveY are swapped due to coordinate system that WPILib uses
        if (_oi.getSlowMode()) {
            vx = vec.x() * Constants.DriveTrain.SLOW_MPS;
            vy = vec.y() * Constants.DriveTrain.SLOW_MPS;
            rot =
                    -rot
                            * Constants.DriveTrain
                                    .SLOW_ANG_VEL; // negative added to flip rotation in slowmode, driver preference
        } else {
            vx = vec.x() * Constants.DriveTrain.MAX_MPS;
            vy = vec.y() * Constants.DriveTrain.MAX_MPS;
            rot = rot * Constants.DriveTrain.MAX_ANG_VEL;
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
        //            } else if (_headingController.getHeadingState() ==
        // SwerveHeadingController.HeadingState.SNAP) {
        //                _headingController.setState(SwerveHeadingController.HeadingState.MAINTAIN);
        //                _headingController.setMaintainHeading(_driveTrain.getHeading());
        ////
        // _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.MAINTAIN);
        ////                _headingController.setGoal(_headingController.getGoal());
        //            } else if (_headingController.getHeadingControllerState() ==
        // HeadingController.HeadingControllerState.OFF) {
        //
        // _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.MAINTAIN);
        //            }
        //        } else {
        //
        // _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.OFF);
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
                _driveTrain.temporaryDisabledHeadingController();
            }
            _lockHeading = true;
        } else {
            _driveTrain.disableHeadingController();
            _lockHeading = false;
        }
        double controllerPower = _driveTrain.getRotationCorrection();
        TrackedObjectInfo closestGameElement;
        if (_endEffector.getConeMode()) {
            closestGameElement = _driveTrain.getClosestCone();
        } else {
            closestGameElement = _driveTrain.getClosestCube();
        }
        double power = 0.0;
        double coneDist = 1.0;
        double elementAngle = 0;
        if (closestGameElement != null) {
            metric("Closest Game Element", closestGameElement.toString());
            power = -_yCoordinateElementController.calculate(closestGameElement.getY());
            coneDist = closestGameElement.getDistance();
            elementAngle = closestGameElement.getAzimuthAngle();
        }
        //        metric("Element Angle", elementAngle);
        metric("Rot+Controller", (rot + controllerPower));
        if (_oi.autoAim()) {
            _driveTrain.setSnapHeading(new Rotation2d(0));
            _driveTrain.setVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            vx * coneDist / 2.0,
                            power,
                            _driveTrain.getRotationCorrection(),
                            _driveTrain.getHeading()));
        } else {
            _driveTrain.setVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            vx, vy, rot + controllerPower, _driveTrain.getHeading()));
        }
        //        _driveTrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
        //                vx,
        //                vy,
        //                rot +controllerPower,
        //                _driveTrain.getHeading()
        //        ));
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _driveTrain.setControlState(ControlState.NEUTRAL);
    }
}
