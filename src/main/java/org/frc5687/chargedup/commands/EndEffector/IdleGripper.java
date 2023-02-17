package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.commands.OutliersCommand;

public class IdleGripper extends OutliersCommand{
    private EndEffector _endEffector;
    public IdleGripper(EndEffector endEffector) {
        _endEffector = endEffector;
        addRequirements(_endEffector);
    }

    @Override
    public void execute() {
        super.execute();
        if (_endEffector.getConeMode()) {
            _endEffector.setRollerSpeed(Constants.EndEffector.ROLLER_CONE_IDLE_SPEED);
        } else {
            _endEffector.setRollerSpeed(Constants.EndEffector.ROLLER_CUBE_IDLE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
}
