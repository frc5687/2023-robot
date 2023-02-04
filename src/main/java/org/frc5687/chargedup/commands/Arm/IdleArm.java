package org.frc5687.chargedup.commands.Arm;

import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;

/* This is to hold the last arm state sent to the controller. */
public class IdleArm extends OutliersCommand {
    private final Arm _arm;
    public IdleArm(Arm arm) {
        _arm = arm;
        addRequirements(_arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        _arm.setNextReference(_arm.getLastState().position, 0);
    }

    @Override
    public void execute() {
        super.execute();
        _arm.setArmVoltage(_arm.getNextVoltage());
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
