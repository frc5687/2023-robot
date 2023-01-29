package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.ExtendingArm;

import edu.wpi.first.math.util.Units;

public class ExtendArm extends OutliersCommand {
    private ExtendingArm _extArm;
    private double _extDistance;

    public ExtendArm(ExtendingArm extArm, double extDistance){
        _extArm = extArm;
        _extDistance = extDistance;
        addRequirements(_extArm);
    }
    @Override
    public void initialize() {
        super.initialize();
        _extArm.setExtArmSetpointDistance(_extDistance);
    }
    
    @Override
    public void execute() {
        super.execute();
        double output = _extArm.getExtArmControllerOutput();
        _extArm.setArmSpeed(output);
    }

    public boolean isFinished(){
        return Math.abs(Units.degreesToRadians(_extDistance) - _extArm.getEncoderRotationRadians()) < Constants.ExtendingArm.EXT_ARM_TOLERANCE;
    }

    public void end(boolean interrupted){
        super.end(interrupted);
    }
}
