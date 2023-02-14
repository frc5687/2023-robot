package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class WaitForManualGripper extends OutliersCommand {

    private final OI _oi;
    private final EndEffector _endEffector;

    public WaitForManualGripper(EndEffector endEffector, OI oi) {
        _oi = oi;
        _endEffector = endEffector;
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
        return _oi.manualGrip() || _endEffector.isRollerStalled();
    }

    @Override
    public void end(boolean interrupted) {
    }
}