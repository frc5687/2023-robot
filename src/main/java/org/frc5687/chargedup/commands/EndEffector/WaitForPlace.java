package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class WaitForPlace extends OutliersCommand {

    private final OI _oi;
    private final EndEffector _endEffector;
    private final boolean _isPlace;

    public WaitForPlace(EndEffector endEffector, OI oi, boolean isPlace) {
        _oi = oi;
        _endEffector = endEffector;
        _isPlace = isPlace;
    }

    @Override
    public void initialize() {
        error("Wait Roller Started");
        super.initialize();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return (_isPlace ? _oi.releaseRoller() : _oi.overrideWrist()) || _endEffector.isRollerStalled();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        error("Wait Roller Ended");
    }
}
