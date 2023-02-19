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
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.lib.control.HeadingController;
import org.frc5687.chargedup.util.Helpers;
import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.math.Vector2d;
import org.frc5687.lib.vision.TrackedObjectInfo;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final SwerveHeadingController _headingController;
    private final OI _oi;
    private boolean _lockHeading;
    private int segmentationArray[] = new int[((int)360/5)];

    public Drive(DriveTrain driveTrain, OI oi) {
        _lockHeading = false;
        _driveTrain = driveTrain;
        _oi = oi;
        _headingController = new SwerveHeadingController(Constants.UPDATE_PERIOD);

        for (int i = 0; i < segmentationArray.length; i++){
            double angle = 360 / segmentationArray.length;
            segmentationArray[i] = (int)angle * i;
        }
        addRequirements(_driveTrain);

    }

    @Override
    public void initialize() {
        _driveTrain.startModules();
        _driveTrain.setFieldRelative(true);
        _headingController.setMaintainHeading(_driveTrain.getHeading());
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
        vx = vec.x() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
        vy = vec.y() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
        rot = rot * Constants.DriveTrain.MAX_ANG_VEL * Constants.DriveTrain.SCALED_ROTATION_INPUT;

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
        TrackedObjectInfo closestGameElement;
        _driveTrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                vx,
                vy,
                rot +controllerPower,
                _driveTrain.getHeading()
        ));
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