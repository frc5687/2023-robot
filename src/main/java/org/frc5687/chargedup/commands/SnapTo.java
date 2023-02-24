package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;

import edu.wpi.first.math.geometry.Rotation2d;

public class SnapTo extends OutliersCommand{
    private DriveTrain _driveTrain;
    private Rotation2d _rotation;
    public SnapTo(DriveTrain driveTrain, Rotation2d rotation) {
        _driveTrain = driveTrain;
        _rotation = rotation;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _driveTrain.setHeadingControllerState(HeadingState.SNAP);
        _driveTrain.setSnapHeading(_rotation);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if (_driveTrain.getHeading().minus(_rotation).getRadians() < 0.01) {
            _driveTrain.setHeadingControllerState(HeadingState.MAINTAIN);
            return true;
        }
        return false;
    }
}
