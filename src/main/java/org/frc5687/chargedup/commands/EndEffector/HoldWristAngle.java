package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class HoldWristAngle extends OutliersCommand {
    private EndEffector _endEffector;
    private double _angle;

    public HoldWristAngle(EndEffector endEffector, double angle) {
        _endEffector = endEffector;
        _angle = angle;
        addRequirements(_endEffector);
    }

    @Override
    public void initialize() {
        _endEffector.setWristSetpointRadians(_angle);
    }

    @Override
    public void execute() {
        _endEffector.setWristSpeed(_endEffector.getWristControllerOutput());
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _endEffector.setWristSpeed(0.0);
    }
}
