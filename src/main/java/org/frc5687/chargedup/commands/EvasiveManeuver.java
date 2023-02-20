package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.Constants.DriveTrain;

public class EvasiveManeuver extends OutliersCommand {
    private final OI _oi;    
    private final DriveTrain _driveTrain;
    EvasiveManeuver(OI oi, DriveTrain driveTrain){
        _oi = oi;
        _driveTrain = driveTrain;
    }

    @Override
    public void execute() {
        new DriveTrajectory(_oi, )
    }
}
