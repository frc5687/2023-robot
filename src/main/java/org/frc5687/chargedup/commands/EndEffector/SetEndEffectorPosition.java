package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.math.util.Units;

public class SetEndEffectorPosition extends OutliersCommand{
    
    private EndEffector _endEffector;
    
    
    private double _wristAngleDegrees;
    private double _gripperAngleDegrees;

    public SetEndEffectorPosition(EndEffector endEffector,
     double wristAngleDegrees, double gripperAngleDegrees ) {
       _endEffector = endEffector;

        _wristAngleDegrees = wristAngleDegrees;
        _gripperAngleDegrees = gripperAngleDegrees;
        addRequirements(_endEffector);
     }
     @Override
     public void initialize() {
         // TODO Auto-generated method stub
         super.initialize();
         _endEffector.setWristSetpointDegrees(_wristAngleDegrees);
         _endEffector.setGripperSetpointDegrees(_gripperAngleDegrees);
     }
     @Override
     public void execute() {
         // TODO Auto-generated method stub
         super.execute();
         double wristOutput = _endEffector.getWristControllerOutput();
         double gripperOutput = _endEffector.getGripperControllerOutput();
         _endEffector.setWristSpeed(wristOutput);
         _endEffector.setGripperSpeed(gripperOutput);
     }
     @Override
     public boolean isFinished() {
         // TODO Auto-generated method stub
         return
          Math.abs(Units.degreesToRadians(_wristAngleDegrees)- _endEffector.getWristAngleRadians()) < Constants.EndEffector.WRIST_TOLERENCE 
          &&  Math.abs(Units.degreesToRadians(_gripperAngleDegrees) - _endEffector.getGripperAngleRadians()) < Constants.EndEffector.GRIPPER_TOLERENCE;
        }
     @Override
     public void end(boolean interrupted) {
         // TODO Auto-generated method stub
         super.end(interrupted);
     }
}
