package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.ExtendingArm;

public class ExtendArm extends OutliersCommand {
    private ExtendingArm _exArm;
    private double _extDistance;

    public ExtendArm(ExtendingArm exArm, double extDistance){
        exArm = _exArm;
        extDistance = _extDistance;
        addRequirements(_exArm);
    }
    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        _exArm.setArmSpeed(_extDistance);
    }


}
