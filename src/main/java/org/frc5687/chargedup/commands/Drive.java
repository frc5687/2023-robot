/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.lib.control.HeadingController;
import org.frc5687.chargedup.util.Helpers;
import org.frc5687.lib.math.Vector2d;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final HeadingController _headingController;
    private final OI _oi;

    private int segmentationArray[] = new int[((int)360/5)]; 

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _headingController = new HeadingController(
                new TrapezoidProfile.Constraints(
                        Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                        Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL
                )
        );
        _oi = oi;

        for (int i = 0; i < segmentationArray.length; i++){
            double angle = 360/segmentationArray.length;
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
        _headingController.setGoal(_driveTrain.getHeading().getRadians());
//        _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
    }

    @Override
    public void execute() {
        metric("Heading Controller State", _headingController.getHeadingControllerState().toString());
        metric("Heading Controller Goal", _headingController.getGoal());
        metric("Heading Controller atGoal", _headingController.isAtGoal(_driveTrain.getHeading().getRadians()));
        if (_oi.zeroIMU()) {
            _driveTrain.zeroGyroscope();
            _headingController.setGoal(_driveTrain.getHeading().getRadians());
        }
        //  driveX and driveY are swapped due to coordinate system that WPILib uses.
        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(_oi.getDriveY(), _oi.getDriveX(), segmentationArray);
        //  driveX and driveY are swapped due to coordinate system that WPILib uses
        double vx = vec.x() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
        double vy = vec.y() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
        double rot = _oi.getRotationX();
        rot = Math.signum(rot) * rot * rot;
        rot = rot * Constants.DriveTrain.MAX_ANG_VEL * Constants.DriveTrain.SCALED_ROTATION_INPUT;

        // 0.01 is the tolerance to start heading controller.
        if (Math.abs(rot) < 0.01) {
            if (_oi.setHeadingNorth()) {
                _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.SNAP);
                _headingController.setGoal(0.0);
            } else if (_oi.setHeadingEast()) {
                _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.SNAP);
                _headingController.setGoal(-Math.PI / 2.0);
            } else if (_oi.setHeadingSouth()) {
                _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.SNAP);
                _headingController.setGoal(Math.PI);
            } else if (_oi.setHeadingWest()) {
                _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.SNAP);
                _headingController.setGoal(Math.PI / 2.0);
            } else if (_headingController.getHeadingControllerState() == HeadingController.HeadingControllerState.SNAP
                    && _headingController.isAtGoal(_driveTrain.getHeading().getRadians())) {
                _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.MAINTAIN);
                _headingController.setGoal(_headingController.getGoal());
            } else if (_headingController.getHeadingControllerState() == HeadingController.HeadingControllerState.OFF) {
                _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.MAINTAIN);
            }
        } else {
            _headingController.setHeadingControllerState(HeadingController.HeadingControllerState.OFF);
            _headingController.reset();
            _headingController.setGoal(_driveTrain.getHeading().getRadians());
        }
        double controllerPower = 0;//_headingController.calculate(_driveTrain.getHeading().getRadians());
        metric("Rot+Controller", (rot + controllerPower));
        _driveTrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                vx,
                vy,
                rot + controllerPower,
                _driveTrain.getHeading()
        ));
        _driveTrain.updateSwerve(new Vector2d(vx, vy), rot);
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
