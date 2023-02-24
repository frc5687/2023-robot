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
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _elevator.setArmSpeed(-0.8);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return _elevator.getInHall();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        _elevator.setArmSpeed(0);
    }
}
