package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.math.util.Units;

public class AutoSetGripperAngle extends OutliersCommand{
    
    private EndEffector _gripper;
    private double _angleDegrees;

    public AutoSetGripperAngle(EndEffector gripper, double angleDegrees){
        _gripper = gripper;
        _angleDegrees = angleDegrees;
        addRequirements(_gripper);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _gripper.setGripperSetpointDegrees(_angleDegrees);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        double output = _gripper.getGripperControllerOutput();
        _gripper.setGripperSetpointDegrees(output);

    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return Math.abs(Units.degreesToRadians(_angleDegrees) - _gripper.getGripperAngleRadians()) < Constants.EndEffector.GRIPPER_TOLERENCE;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
