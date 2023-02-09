package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.math.util.Units;

public class AutoSetRollerSpeed extends OutliersCommand{
    
    private EndEffector _roller;
    private double _speed;

    private long _timeout;

    public AutoSetRollerSpeed(EndEffector roller, double speed){
        _roller = roller;
        _speed = speed;
        addRequirements(_roller);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _timeout = System.currentTimeMillis() + Constants.EndEffector.GRIPPER_TIMEOUT;
       /*  _gripper.setGripperSetpointRadians(_angle);
        metric("Setpoint", _angle);*/
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
       /*  double output = _gripper.getGripperControllerOutput();
        metric("output", output);*/
        _roller.setRollerSpeed(_speed);

    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
