/* Team 5687 (C)2021-2022 */
package org.frc5687.chargedup.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
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
        double vx = _oi.getDriveY() * Constants.DriveTrain.MAX_MPS;
        double vy = _oi.getDriveX() * Constants.DriveTrain.MAX_MPS;
        double rot = _oi.getRotationX() * Constants.DriveTrain.MAX_ANG_VEL;

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
