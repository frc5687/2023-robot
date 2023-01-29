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
        _extArm.setArmSpeed(_oi.getExtArmY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
