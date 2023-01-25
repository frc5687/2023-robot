package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.ExtendingArm;

public class ExtendArm extends OutliersCommand {
    private ExtendingArm exArm;
    private double extDistance;
    private int count;

    public ExtendArm(ExtendingArm _exArm, double _extDistance){
        _exArm = exArm;
        _extDistance = extDistance;

        _exArm.setArmSpeed(extDistance);
    }
    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
    }


}
