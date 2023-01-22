package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.math.util.Units;

public class AutoSetWristAngle extends OutliersCommand {
    
    private EndEffector _wrist;
    private double _angleDegrees;

    public AutoSetWristAngle(EndEffector wrist, double angleDegrees){
        _wrist = wrist;
        _angleDegrees = angleDegrees;
        addRequirements(_wrist);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _wrist.setWristSetpointDegrees(_angleDegrees);

    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        double output = _wrist.getWristControllerOutput();
        _wrist.setWristSpeed(output);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return Math.abs( Units.degreesToRadians(_angleDegrees) -_wrist.getWristAngleRadians()) < Constants.EndEffector.WRIST_TOLERENCE;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
