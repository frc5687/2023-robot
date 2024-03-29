package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class AutoSetWristAngle extends OutliersCommand {

    private EndEffector _wrist;
    private double _angle;

    public AutoSetWristAngle(EndEffector wrist, double angle) {
        _wrist = wrist;
        _angle = angle;
        addRequirements(_wrist);
    }

    @Override
    public void initialize() {
        super.initialize();
        _wrist.setWristSetpointRadians(_angle);
        metric("setpoint", _angle);
    }

    @Override
    public void execute() {
        super.execute();
        double output = _wrist.getWristControllerOutput();
        _wrist.setWristSpeed(output);
        metric("output", output);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(_angle - _wrist.getWristAngleRadians()) < Constants.EndEffector.WRIST_TOLERENCE;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _wrist.setWristSpeed(0);
    }
}
