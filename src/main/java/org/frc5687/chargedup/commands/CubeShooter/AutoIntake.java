package org.frc5687.chargedup.commands.CubeShooter;

import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;

public class AutoIntake extends OutliersCommand {
    private CubeShooter _cubeShooter;
    public AutoIntake(CubeShooter shooter) {
        _cubeShooter = shooter;
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
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _cubeShooter.setShooterSpeed(0);
    }
}
