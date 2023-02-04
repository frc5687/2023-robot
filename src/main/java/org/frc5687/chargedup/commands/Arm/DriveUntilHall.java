package org.frc5687.chargedup.commands.Arm;

import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;

public class DriveUntilHall extends OutliersCommand{
    private Arm _arm;
    private boolean _reversed;

    public DriveUntilHall(Arm arm, boolean reversed) {
        _arm = arm;
        _reversed = reversed;
        addRequirements(_arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        super.execute();
        _arm.setNextReference(_arm.getArmAngleRadians(), 0.0);
        if (_reversed) {
            _arm.setArmSpeed(-0.2);
        }
            _arm.setArmSpeed(0.2);
    }

    public boolean isFinished() {
        _arm.zeroEncoder();
        return _arm.getUpperHall();
    }
}
