package org.frc5687.chargedup.commands.Elevator;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Elevator;

public class ManualExtendElevator extends OutliersCommand {
    private Elevator _elevator;
    private OI _oi;

    public ManualExtendElevator(Elevator elevator, OI oi) {
        _elevator = elevator;
        _oi = oi;
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
        double speed = _oi.getExtArmY();
        if (_elevator.getInHall() && speed < 0) {
            _elevator.setArmSpeed(0);
        } else if (_elevator.getOutHall() && speed > 0) {
            _elevator.setArmSpeed(0);
        } else {
            _elevator.setArmSpeed(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
