/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final PIDController _angleController;
    private double heading;
    private final OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _angleController = new PIDController(3, 0, 0);
        heading = _driveTrain.getHeading().getRadians();
        addRequirements(_driveTrain);
        //        logMetrics("vx","vy");
        //        enableMetrics();

    }

    @Override
    public void initialize() {
        _driveTrain.startModules();
        _driveTrain.setFieldRelative(true);
//        _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
    }

    @Override
    public void execute() {
        //  driveX and driveY are swapped due to coordinate system that WPILib uses.
        double vx = _oi.getDriveY() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
        double vy = _oi.getDriveX() * Constants.DriveTrain.MAX_MPS * Constants.DriveTrain.SCALED_TRANSLATION_INPUT;
        double rot = _oi.getRotationX();
        rot = Math.signum(rot) * rot * rot;
        rot = rot * Constants.DriveTrain.MAX_ANG_VEL * Constants.DriveTrain.SCALED_ROTATION_INPUT;
        if (Math.abs(rot) < 0.05) {
            rot = _angleController.calculate(_driveTrain.getHeading().getRadians(), heading);
        } else {
            heading = _driveTrain.getHeading().getRadians();
        }
        _driveTrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                vx,
                vy,
                rot,
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
