package org.frc5687.chargedup.commands.CubeShooter;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;

public class AutoIntake extends OutliersCommand {
    private CubeShooter _cubeShooter;
    private OI _oi;
    public AutoIntake(CubeShooter shooter, OI oi) {
        _cubeShooter = shooter;
        _oi = oi;
        addRequirements(_cubeShooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _cubeShooter.setShooterSpeed(-.5);
    }

    @Override
    public boolean isFinished() {
        return !_oi.getCubeIntake() /*|| detecting cube in*/;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _cubeShooter.setShooterSpeed(0);
    }
}
