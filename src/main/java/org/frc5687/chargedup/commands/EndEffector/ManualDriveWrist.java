package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class ManualDriveWrist extends OutliersCommand {
    private EndEffector _wrist;
    private OI _oi;

    public ManualDriveWrist(EndEffector wrist, OI oi) {
        _wrist = wrist;
        _oi = oi;
        addRequirements(_wrist);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speed = _oi.getWristSpeed();
        _wrist.setWristSpeed(speed / 2.0);
        metric("wrist speed", speed / 2.0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
