package org.frc5687.chargedup.commands.Arm;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;

public class ManualDriveArm extends OutliersCommand {

    private Arm _arm;
    private OI _oi;

    public ManualDriveArm(Arm arm, OI oi) {
        _arm = arm;
        _oi = oi;
        addRequirements(_arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        super.execute();
        _arm.setArmSpeed(_oi.getArmY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}