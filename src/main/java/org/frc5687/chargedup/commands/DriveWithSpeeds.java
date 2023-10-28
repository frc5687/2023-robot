package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.DriveTrain.Mode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveWithSpeeds extends OutliersCommand{

    private DriveTrain _driveTrain;
    private double _vx;
    private double _vy;
    
    public DriveWithSpeeds(DriveTrain driveTrain, double vx, double vy) {
        _driveTrain = driveTrain;
        _vx = vx;
        _vy = vy;
    }

    @Override
    public void execute() {
        super.execute();    
        _driveTrain.setMode(Mode.NORMAL);
        _driveTrain.setKinematicLimits(Constants.DriveTrain.KINEMATIC_LIMITS);
        _driveTrain.setVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                _vx, _vy, 0, _driveTrain.getHeading()
            )
        );
    }
}
