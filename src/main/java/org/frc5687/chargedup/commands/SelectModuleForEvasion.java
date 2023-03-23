package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Translation2d;

public class SelectModuleForEvasion extends OutliersCommand{
    private final DriveTrain _driveTrain;
    private boolean _clockwise;

    public SelectModuleForEvasion(DriveTrain driveTrain, boolean clockwise){
        _driveTrain = driveTrain;
        _clockwise = clockwise;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
    
    @Override
    public void execute() {
        _driveTrain.determineCORForEvasion();
        _driveTrain.setCenterOfRotation(_clockwise ? _driveTrain.getClockwiseCOR() : _driveTrain.getCounterClockwiseCOR());
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.setCenterOfRotation(new Translation2d());
        super.end(interrupted);
    }
}
