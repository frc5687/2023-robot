package org.frc5687.chargedup.commands.CubeShooter;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;

public class AutoRotateWrist extends OutliersCommand {
    private CubeShooter _cubeShooter;
    private double _wristAngle;

    public AutoRotateWrist(CubeShooter cubeShooter, double angle) {
        _cubeShooter = cubeShooter;
        _wristAngle = angle;
        addRequirements(_cubeShooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        error("Starting wrist cube");
    }

    @Override
    public void execute() {
        super.execute();
        _cubeShooter.setWristAngle(_wristAngle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(_wristAngle - _cubeShooter.getWristAngleRadians())
                < Constants.CubeShooter.WRIST_ANGLE_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        error("Finishing cube wrist");
    }
}
