package org.frc5687.chargedup.commands.Arm;

import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;

/* This is to hold the last arm state sent to the controller. */
public class HoldArm extends OutliersCommand {
    private final Arm _arm;
    private double _angle;

    public HoldArm(Arm arm, double angle) {
        _arm = arm;
        _angle = angle;
        addRequirements(_arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        _arm.setNextReference(_angle, 0);
    }

    @Override
    public void execute() {
        super.execute();
        _arm.setArmVoltage(_arm.getNextVoltage() + _arm.armFeedForward());
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _arm.setArmVoltage(0);
    }
}
