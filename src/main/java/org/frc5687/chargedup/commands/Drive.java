/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.util.Helpers;
import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.lib.math.Vector2d;
import org.frc5687.lib.vision.TrackedObjectInfo;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final EndEffector _endEffector;
    //    private final HeadingController _headingController;
    private final PIDController _yCordinateElementController;
    private final OI _oi;
    private boolean _lockHeading;
    private int segmentationArray[] = new int[((int) 360 / 5)];

    public Drive(DriveTrain driveTrain, EndEffector endEffector, OI oi) {
        _lockHeading = false;
        _driveTrain = driveTrain;
        _endEffector = endEffector;
        _oi = oi;
        _yCordinateElementController = new PIDController(3.0, 0.0, 0.3);
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
        //        _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
    }

    @Override
    public void execute() {
        if (_oi.zeroIMU()) {
            _driveTrain.zeroGyroscope();
            _driveTrain.setHeadingControllerState(SwerveHeadingController.HeadingState.OFF);
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
            _driveTrain.setSlowMode(true);
            vx = vec.x() * Constants.DriveTrain.SLOW_MPS;
            vy = vec.y() * Constants.DriveTrain.SLOW_MPS;
            rot =
                    -rot
                            * Constants.DriveTrain
                                    .SLOW_ANG_VEL; // negative added to flip rotation in slowmode, driver preference
        } else {
            _driveTrain.setSlowMode(false);
            vx = vec.x() * Constants.DriveTrain.MAX_MPS;
            vy = vec.y() * Constants.DriveTrain.MAX_MPS;
            rot = rot * Constants.DriveTrain.MAX_ANG_VEL;
        }

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
        TrackedObjectInfo closestGameElement;
        if (_endEffector != null && _endEffector.getConeMode()) {
            closestGameElement = _driveTrain.getClosestCone();
        } else {
            closestGameElement = _driveTrain.getClosestCube();
        }
    
        double power = 0.0;
        // settings
        double coneDist = vx;
        //double elementAngle = 0;
        if (closestGameElement != null) {
            metric("Closest Game Element", closestGameElement.toString());
            power = -_yCordinateElementController.calculate(closestGameElement.getY());
            coneDist = closestGameElement.getDistance();
         //   elementAngle = closestGameElement.getAzimuthAngle();
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
    }
}
