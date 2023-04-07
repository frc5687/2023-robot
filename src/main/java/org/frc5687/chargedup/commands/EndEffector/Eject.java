package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;

public class Eject extends OutliersCommand {
    private EndEffector _endEffector;
    private long _timeout;

    public Eject(EndEffector endEffector){
        _endEffector = endEffector;
    }

    @Override
    public void initialize() {
        super.initialize();
        _timeout = System.currentTimeMillis() + 500;
    }

    @Override
    public void execute() {
        super.execute();
        if (_endEffector.getConeMode()){
        _endEffector.setRollerSpeed(Constants.EndEffector.GRIPPER_OUT_SPEED);
        } else {
            _endEffector.setRollerSpeed(Constants.EndEffector.GRIPPER_IN_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return _timeout < System.currentTimeMillis();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
