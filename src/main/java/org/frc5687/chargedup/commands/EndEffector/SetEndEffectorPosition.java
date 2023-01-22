package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.math.util.Units;

public class SetEndEffectorPosition extends OutliersCommand{
    
    private EndEffector _wrist;
    private EndEffector _gripper;
    
    private double _wristAngleDegrees;
    private double _gripperAngleDegrees;

    public SetEndEffectorPosition(EndEffector wrist, EndEffector gripper,
     double wristAngleDegrees, double gripperAngleDegrees ) {
        _wrist = wrist;
        _gripper = gripper;

        _wristAngleDegrees = wristAngleDegrees;
        _gripperAngleDegrees = gripperAngleDegrees;
        addRequirements(_wrist, gripper);
     }
     @Override
     public void initialize() {
         // TODO Auto-generated method stub
         super.initialize();
         _wrist.setWristSetpointDegrees(_wristAngleDegrees);
         _gripper.setGripperSetpointDegrees(_gripperAngleDegrees);
     }
     @Override
     public void execute() {
         // TODO Auto-generated method stub
         super.execute();
         double wristOutput = _wrist.getWristControllerOutput();
         double gripperOutput = _gripper.getGripperControllerOutput();
         _wrist.setWristSpeed(wristOutput);
         _gripper.setGripperSpeed(gripperOutput);
     }
     @Override
     public boolean isFinished() {
         // TODO Auto-generated method stub
         return Math.abs(Units.degreesToRadians(_wristAngleDegrees)- _wrist.getWristAngleRadians()) < Constants.EndEffector.WRIST_TOLERENCE &&  Math.abs(Units.degreesToRadians(_gripperAngleDegrees) - _gripper.getGripperAngleRadians()) < Constants.EndEffector.GRIPPER_TOLERENCE;
        }
     @Override
     public void end(boolean interrupted) {
         // TODO Auto-generated method stub
         super.end(interrupted);
     }
}
