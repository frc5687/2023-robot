package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.math.util.Units;

public class AutoSetGripperAngle extends OutliersCommand{
    
    private EndEffector _gripper;
    private double _angle;

    private long _timeout;

    public AutoSetGripperAngle(EndEffector gripper, double angle){
        _gripper = gripper;
        _angle = angle;
        addRequirements(_gripper);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _timeout = System.currentTimeMillis() + Constants.EndEffector.GRIPPER_TIMEOUT;
        _gripper.setGripperSetpointRadians(_angle);
        metric("Setpoint", _angle);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        double output = _gripper.getGripperControllerOutput();
        metric("output", output);
        _gripper.setGripperSpeed(output);

    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return Math.abs(_angle - _gripper.getGripperAngleRadians()) < Constants.EndEffector.GRIPPER_TOLERENCE || _timeout < System.currentTimeMillis();
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
