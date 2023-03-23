/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.DriveTrain.Mode;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.DriveTrain.ControlState;
import org.frc5687.lib.control.HeadingController;
import org.frc5687.chargedup.util.Helpers;
import org.frc5687.chargedup.util.OutliersContainer.IdentityMode;
import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
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
        _yCoordinateElementController = new PIDController(2.5, 0.0, 0.3);
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
        // _driveTrain.setFieldRelative(true);
//        _headingController.setGoal(_driveTrain.getHeading().getRadians());
        // _headingController.setMaintainHeading(_driveTrain.getHeading());
        _driveTrain.setPrevVelocity(_driveTrain.getDesiredChassisSpeeds());
        // _driveTrain.determineCORForEvasion(_oi.getTranslationVector());
       _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
    }

    @Override
    public void execute() {
        if (_oi.zeroIMU()) {
            _driveTrain.zeroGyroscope();
            _driveTrain.setHeadingControllerState(SwerveHeadingController.HeadingState.OFF);
        }

        if ((_oi.getTranslationVector().x() - _driveTrain.getPrevVector2d().x() > Constants.DriveTrain.PREV_VECTOR_CHANGE_X 
        || _oi.getTranslationVector().y() - _driveTrain.getPrevVector2d().y() > Constants.DriveTrain.PREV_VECTOR_CHANGE_Y) 
         && !_oi.getTranslationVector().equals(new Vector2d(0, 0))){
            _driveTrain.setPrevVector2d(_oi.getTranslationVector().x(), _oi.getTranslationVector().y());
        }
        // if (_oi.setCORLeft() || _oi.setCORRight())
        // _driveTrain.determineCORForEvasion();

        if (_oi.setCORLeft()){
            _driveTrain.setCenterOfRotation(_driveTrain.getCounterClockwiseCOR());
        }
        if (_oi.setCORRight()){
            _driveTrain.setCenterOfRotation(_driveTrain.getClockwiseCOR());
        } else {
            _driveTrain.setCenterOfRotation(new Translation2d());
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
        if (rot == 0 && _driveTrain.getHeadingControllerState() != HeadingState.SNAP) {
            if (!_lockHeading) {
                _driveTrain.temporaryDisabledHeadingController();
            }
            _lockHeading = true;
        } else if (_driveTrain.getHeadingControllerState() != HeadingState.SNAP) {
            _driveTrain.disableHeadingController();
            _lockHeading = false;
        }

        double controllerPower = _driveTrain.getRotationCorrection();
        //        metric("Element Angle", elementAngle);
        metric("Rot+Controller", (rot + controllerPower));
        // if (_driveTrain.getIdentityMode() == IdentityMode.competition){
        if (_oi.autoAim()) {
            _driveTrain.setMode(Mode.VISION);
            _driveTrain.setKinematicLimits(Constants.DriveTrain.VISION_KINEMATIC_LIMITS);
            vx = vec.x() * Constants.DriveTrain.VISION_KINEMATIC_LIMITS.maxDriveVelocity;
            vy = vec.y() * Constants.DriveTrain.VISION_KINEMATIC_LIMITS.maxDriveVelocity;
            rot = rot * Constants.DriveTrain.VISION_KINEMATIC_LIMITS.maxSteeringVelocity;
            TrackedObjectInfo closestGameElement;
            if (_endEffector != null && _endEffector.getConeMode()) {
                closestGameElement = _driveTrain.getClosestCone();
            } else {
                closestGameElement = _driveTrain.getClosestCube();
            }
            double power = 0.0;
            // settings
            double coneDist = vx;
            boolean targetInTolerance;
            // double elementAngle = 0;
            if (closestGameElement != null) {
                metric("Closest Game Element", closestGameElement.toString());
                coneDist = closestGameElement.getDistance();
                targetInTolerance =
                        (closestGameElement.getZ() > Units.inchesToMeters(37)
                                && closestGameElement.getZ() < Units.inchesToMeters(65));
                if (targetInTolerance) {
                    power = -_yCoordinateElementController.calculate(closestGameElement.getY());
                }
                //   elementAngle = closestGameElement.getAzimuthAngle();
            }
            _driveTrain.setMaintainHeading(new Rotation2d(0));
            _driveTrain.setVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            vx * coneDist / 2.0,
                            power,
                            _driveTrain.getRotationCorrection(),
                            _driveTrain.getHeading()));
        } else if (_oi.getSlowMode()) {
            _driveTrain.setMode(Mode.SLOW);
            _driveTrain.setKinematicLimits(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS);
            vx = vec.x() * Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity;
            vy = vec.y() * Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity;
            rot =
                    -rot
                            * Constants.DriveTrain
                                    .SLOW_ANG_VEL; // negative added to flip rotation in slowmode, driver preference
            _driveTrain.setVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            vx, vy, rot + controllerPower, _driveTrain.getHeading()));
        } else {
            _driveTrain.setMode(Mode.NORMAL);
            _driveTrain.setKinematicLimits(Constants.DriveTrain.KINEMATIC_LIMITS);
            vx = vec.x() * Constants.DriveTrain.MAX_MPS;
            vy = vec.y() * Constants.DriveTrain.MAX_MPS;
            rot = rot * Constants.DriveTrain.MAX_ANG_VEL;
            _driveTrain.setVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            vx, vy, rot + controllerPower, _driveTrain.getHeading()));
        }
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

    // public Drive(DriveTrain driveTrain, OI oi){
    //     _driveTrain = driveTrain;
    //     _oi = oi;
    //     _endEffector = null;
    //     _yCordinateElementController = new PIDController(2.5, 0, 0.3);
    // }

    
}