package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.subsystems.Lights.AnimationType;

public class WaitForManualGripper extends OutliersCommand {

    private final OI _oi;
    private final Lights _lights;
    public WaitForManualGripper(OI oi, Lights lights) {
        _oi = oi;
        _lights = lights;
        addRequirements(_lights);
    }

    @Override
    public void initialize() {
        super.initialize();
        _lights.switchAnimation(AnimationType.STATIC);
    }   

    @Override
    public void execute() {
        _lights.setColor(Constants.CANdle.GREEN);
    }

    @Override
    public boolean isFinished() {
        return _oi.manualGrip();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
