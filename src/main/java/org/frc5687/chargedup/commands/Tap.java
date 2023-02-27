package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.subsystems.DriveTrain;

public class Tap extends OutliersCommand {

    private DriveTrain _driveTrain;
    private boolean _isToTheRight;

    /**
     * Increments the DriveTrain's heading controller either left or right a bit based on
     * isToTheRight.
     *
     * @param driveTrain The DriveTrain we use
     * @param isToTheRight If true, tap right. If false, tap left.
     */
    public Tap(DriveTrain driveTrain, boolean isToTheRight) {
        _driveTrain = driveTrain;
        _isToTheRight = isToTheRight;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        if (_isToTheRight) {
            _driveTrain.incrementHeadingControllerAngle();
        } else {
            _driveTrain.decrementHeadingControllerAngle();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
