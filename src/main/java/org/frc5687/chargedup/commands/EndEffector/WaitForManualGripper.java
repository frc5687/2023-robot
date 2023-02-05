package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class WaitForManualGripper extends OutliersCommand {

    private final OI _oi;

    public WaitForManualGripper(OI oi) {
        _oi = oi;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return _oi.manualGrip();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
