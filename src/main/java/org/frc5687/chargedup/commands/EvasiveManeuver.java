package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.lib.math.Vector2d;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;

public class EvasiveManeuver extends OutliersCommand {
    private final OI _oi;    
    private final DriveTrain _driveTrain;
    

    public EvasiveManeuver(OI oi, DriveTrain driveTrain){
        _oi = oi;
        _driveTrain = driveTrain;
    }
@Override
public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    Vector2d currentTransVec = new Vector2d(_oi.getDriveX(), _oi.getDriveY());
    Translation2d rotVec = new Translation2d(1/_oi.getCOR(), 1/_oi.getCOR());
    _driveTrain.updateSwerve(currentTransVec, 1, rotVec)/*<currentTransVec, _oi.getRotationX, _oi.getCOR>*/;
}
}
