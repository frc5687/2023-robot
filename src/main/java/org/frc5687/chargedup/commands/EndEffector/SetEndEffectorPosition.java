package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SetEndEffectorPosition extends OutliersCommand {

    private EndEffector _endEffector;

    private double _wristAngle;
    private double _gripperSpeed;

    public SetEndEffectorPosition(EndEffector endEffector, double wristAngle, double gripperSpeed) {
        _endEffector = endEffector;

        _wristAngle = wristAngle;
        _gripperSpeed = gripperSpeed;
        addRequirements(_endEffector);
    }

    @Override
    public void initialize() {
        super.initialize();
        _endEffector.setWristSetpointRadians(_wristAngle);
        //  _endEffector.setGripperSpeed(_gripperSpeed);
    }

    @Override
    public void execute() {
        super.execute();
        double wristOutput = _endEffector.getWristControllerOutput();
        // double gripperOutput = _endEffector.getGripperControllerOutput();
        _endEffector.setWristSpeed(wristOutput);
        _endEffector.setRollerSpeed(_gripperSpeed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(_wristAngle - _endEffector.getWristAngleRadians())
                < Constants.EndEffector.WRIST_TOLERENCE;
        // &&  Math.abs(_gripperSpeed - _endEffector.getGripperAngleRadians()) <
        // Constants.EndEffector.GRIPPER_TOLERENCE;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        error("end wrist");
        _endEffector.setWristSpeed(0.0);
    }
}
