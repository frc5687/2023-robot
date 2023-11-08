package org.frc5687.chargedup.commands.Arm;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.NewArm;

public class SetArmAngle extends OutliersCommand{

    private NewArm _arm;
    private double _angle;

    public SetArmAngle(NewArm arm, double angle) {
        _arm = arm;
        _angle = angle;
    }

    @Override
    public void initialize() {
        _arm.setArmAngle(_angle);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return Math.abs(_arm.getArmAngleRadians() - _angle) < Constants.Arm.ANGLE_TOLERANCE;
    }
}
