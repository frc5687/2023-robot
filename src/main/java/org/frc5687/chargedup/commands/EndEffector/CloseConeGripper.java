package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.commands.OutliersCommand;

public class CloseConeGripper extends OutliersCommand{
    private EndEffector _endEffector;
    private long _timeout;

    public CloseConeGripper(EndEffector endEffector){
        _endEffector = endEffector;
        addRequirements(_endEffector);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _timeout = System.currentTimeMillis() + 1600;
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _endEffector.setRollerSpeed(Constants.EndEffector.GRIPPER_IN_SPEED);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return _endEffector.isRollerStalled();
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        _endEffector.setRollerSpeed(0);
    }
}
