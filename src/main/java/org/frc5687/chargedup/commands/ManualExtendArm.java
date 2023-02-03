package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.ExtendingArm;

public class ManualExtendArm extends OutliersCommand{
    private ExtendingArm _extArm;
    private OI _oi;


    public ManualExtendArm(ExtendingArm extArm, OI oi){
        _extArm = extArm;
        _oi = oi;
        addRequirements(_extArm);
    }
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        double speed = _oi.getExtArmY();
        if(_extArm.getInHall() && speed < 0){
            _extArm.setArmSpeed(0);
        }
        else if (_extArm.getOutHall() && speed > 0){
            _extArm.setArmSpeed(0);
        }
        else {
            _extArm.setArmSpeed(speed);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
