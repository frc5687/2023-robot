package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class CloseConeGripper extends OutliersCommand {
    private EndEffector _endEffector;

    public CloseConeGripper(EndEffector endEffector) {
        _endEffector = endEffector;
        addRequirements(_endEffector);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        _endEffector.setRollerSpeed(Constants.EndEffector.GRIPPER_IN_SPEED);
    }

    @Override
    public boolean isFinished() {
        return _endEffector.isRollerStalled();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _endEffector.setRollerSpeed(0);
    }
}
