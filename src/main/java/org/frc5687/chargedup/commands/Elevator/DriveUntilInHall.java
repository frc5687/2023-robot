package org.frc5687.chargedup.commands.Elevator;

import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Elevator;

public class DriveUntilInHall extends OutliersCommand {
    private Elevator _elevator;

    public DriveUntilInHall(Elevator elevator) {
        _elevator = elevator;
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        _elevator.setHasZeroed(false);
    }

    @Override
    public void execute() {
        super.execute();
        _elevator.setArmSpeed(-0.3);
    }

    @Override
    public boolean isFinished() {
        return _elevator.getInHall();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _elevator.setArmSpeed(0);
    }
}
