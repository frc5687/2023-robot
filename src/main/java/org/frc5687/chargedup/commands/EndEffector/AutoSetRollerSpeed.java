package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;


public class AutoSetRollerSpeed extends OutliersCommand{
    
    private EndEffector _roller;
    private double _speed;

    private long _timeout;
    private boolean _withTimeout;

    public AutoSetRollerSpeed(EndEffector roller, double speed, boolean withTimeout){
        _roller = roller;
        _speed = speed;
        _withTimeout = withTimeout;
        addRequirements(_roller);
    }
    @Override
    public void initialize() {
        super.initialize();
        _timeout = System.currentTimeMillis() + Constants.EndEffector.GRIPPER_TIMEOUT;
       /*  _gripper.setGripperSetpointRadians(_angle);
        metric("Setpoint", _angle);*/
    }
    @Override
    public void execute() {
        super.execute();
       /*  double output = _gripper.getGripperControllerOutput();
        metric("output", output);*/
        _roller.setRollerSpeed(_speed);

    }
    @Override
    public boolean isFinished() {
        if (_withTimeout) {
            return _timeout < System.currentTimeMillis();
        }
       return _roller.isRollerStalled();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
