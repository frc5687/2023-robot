package org.frc5687.chargedup.commands.EndEffector;

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
        _endEffector.setRollerSpeed(-0.25); // -0.25 while sucking in a cone should not normally exceed 10 amps on the intake
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
}
