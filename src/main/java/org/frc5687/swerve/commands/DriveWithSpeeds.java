package org.frc5687.swerve.commands;

import org.frc5687.swerve.Constants;
import org.frc5687.swerve.subsystems.DriveTrain;
import org.frc5687.swerve.subsystems.DriveTrain.Mode;

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
        _driveTrain.setKinematicLimits(_driveTrain.isLowGear() ? Constants.DriveTrain.LOW_KINEMATIC_LIMITS :
         Constants.DriveTrain.HIGH_KINEMATIC_LIMITS);
        _driveTrain.setVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                _vx, _vy, 0, _driveTrain.getHeading()
            )
        );
    }
}
