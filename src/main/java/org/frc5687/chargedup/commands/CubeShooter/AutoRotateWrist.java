package org.frc5687.chargedup.commands.CubeShooter;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;

public class AutoRotateWrist extends OutliersCommand {
    private CubeShooter _cubeShooter;
    private double _wristRotations;

    public AutoRotateWrist(CubeShooter cubeShooter, double wristRotations){
        _cubeShooter = cubeShooter;
        _wristRotations = wristRotations;
        addRequirements(_cubeShooter);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _cubeShooter.setWristPosition(_wristRotations);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return Math.abs(_wristRotations - _cubeShooter.getWristAngleRadians()) < Constants.CubeShooter.WRIST_ANGLE_TOLERANCE;
    }

}
