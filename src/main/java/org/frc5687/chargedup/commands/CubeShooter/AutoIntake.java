package org.frc5687.chargedup.commands.CubeShooter;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;

public class AutoIntake extends OutliersCommand {
    private CubeShooter _cubeShooter;
    private boolean _override;
    private long _timer;
    private boolean _timerStarted;

    public AutoIntake(CubeShooter shooter, boolean override) {
        _cubeShooter = shooter;
        _override = override;
        addRequirements(_cubeShooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        _cubeShooter.setShooterSpeed(-.4);
        _cubeShooter.setWristAngle(Constants.CubeShooter.INTAKE_ANGLE);
        // _timer = System.currentTimeMillis() + 50;
    }

    @Override
    public void execute() {
        if (_cubeShooter.isCubeDetected() && !_timerStarted){
            _timerStarted = true;
            _timer = System.currentTimeMillis() + 90;
        } else if (_timerStarted && !_cubeShooter.isCubeDetected()){
            _timerStarted = false;
        }
    }

    @Override
    public boolean isFinished() {
        if (_timer < System.currentTimeMillis() && _cubeShooter.isCubeDetected() && _timerStarted){
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _cubeShooter.setShooterSpeed(0);
        _cubeShooter.setWristAngle(Constants.CubeShooter.IDLE_ANGLE);
    }
}
