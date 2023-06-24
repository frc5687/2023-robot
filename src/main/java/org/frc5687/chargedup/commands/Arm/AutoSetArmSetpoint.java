package org.frc5687.chargedup.commands.Arm;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;

import com.ctre.phoenix.sensors.CANCoder;

public class AutoSetArmSetpoint extends OutliersCommand {

    private final Arm _arm;

    // We need a variable to place the goal of our arm
    private final double _angle;

    // In the constructor we need the Arm subsystem and the angle we would like to go to.
    public AutoSetArmSetpoint(Arm arm, double angle) {
        _arm = arm;
        _angle = angle;
        addRequirements(_arm);
    }

    @Override
    public void initialize() {
        _arm.setArmAngle(_angle);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // This command should finish when the arm has done what we want.
//        return Math.abs(goal.position - _arm.getArmAngleRadians()) < Constants.Arm.ANGLE_TOLERANCE;
        return Math.abs(_angle - _arm.getArmAngleRadians()) < Constants.Arm.ANGLE_TOLERANCE;
        // super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        error("Ending arm");
        super.end(interrupted);
    }
}
