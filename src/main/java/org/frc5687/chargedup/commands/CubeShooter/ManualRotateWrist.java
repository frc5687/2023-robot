package org.frc5687.chargedup.commands.CubeShooter;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;

public class ManualRotateWrist extends OutliersCommand {
    private CubeShooter _cubeShooter;
    private OI _oi;

    public ManualRotateWrist(CubeShooter cubeShooter, OI oi) {
        _cubeShooter = cubeShooter;
        _oi = oi;
        addRequirements(_cubeShooter);
    }

    @Override
    public void initialize() {
       
        super.initialize();
    }

    @Override
    public void execute() {
      
        super.execute();
        double speed = _oi.getCSWrist();
        _cubeShooter.setWristSpeed(speed);
        //        _cubeShooter.setShooterSpeed(speed);
    }

    @Override
    public boolean isFinished() {
      
        return false;
    }
}
