package org.frc5687.chargedup.commands.CubeShooter;

import org.frc5687.chargedup.Constants;
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
        _cubeShooter.setShooterSpeed(-.5);
        _cubeShooter.setWristAngle(Constants.CubeShooter.INTAKE_ANGLE);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return _cubeShooter.isCubeDetected();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _cubeShooter.setShooterSpeed(0);
        _cubeShooter.setWristAngle(Constants.CubeShooter.IDLE_ANGLE);
    }
}