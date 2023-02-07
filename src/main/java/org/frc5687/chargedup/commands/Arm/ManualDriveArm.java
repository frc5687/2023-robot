package org.frc5687.chargedup.commands.Arm;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;

public class ManualDriveArm extends OutliersCommand {

    private final Arm _arm;
    private final OI _oi;

    public ManualDriveArm(Arm arm, OI oi) {
        _arm = arm;
        _oi = oi;
        addRequirements(_arm);
    }

    @Override
    public void initialize() {
        // _arm.setNextReference(_arm.getLastState().position, 0);
    }

    @Override
    public void execute() {
        super.execute();
        // set the arm reference to be current arm angle and zero velocity.
        if (_arm.getLowerHall() && (_oi.getArmY() < 0)) {
            _arm.setArmSpeed(0);
        // } else if (_oi.getArmY() == 0) {
        //     _arm.setArmVoltage(_arm.getNextVoltage());
        } else {
            _arm.setNextReference(_arm.getArmAngleRadians(), 0);
            _arm.setArmSpeed(_oi.getArmY());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
