package org.frc5687.chargedup.commands.CubeShooter;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;

public class Shoot extends OutliersCommand {

    private long _timeout;
    private ShootingState _state;
    private CubeShooter _cubeShooter;
    private double _speed;
    private double _angle;
    private boolean _finished;

    public Shoot(CubeShooter shooter, double speed, double angle) {
        _cubeShooter = shooter;
        _speed = speed;
        _angle = angle;
        _state = ShootingState.INITIALIZE;
        _finished = false;
        addRequirements(_cubeShooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        _state = ShootingState.INITIALIZE;
        _finished = false;
        _timeout = System.currentTimeMillis() + 750;
        _cubeShooter.setWristAngle(_angle);
    }

    @Override
    public void execute() {
        switch (_state) {
            case INITIALIZE:
//                _cubeShooter.setShooterSpeed(-0.5);
               if (Math.abs(_cubeShooter.getWristEncoderRotation() - _angle) < Constants.CubeShooter.WRIST_ANGLE_TOLERANCE) {
                   _timeout = System.currentTimeMillis() + 750;
                   _state = ShootingState.REVVING_UP;
               }
                // _state = ShootingState.REVVING_UP;
                break;
            case REVVING_UP:
                _cubeShooter.setShooterSpeed(_speed);
                if (_timeout < System.currentTimeMillis()) {
                    _state = ShootingState.UP_TO_SPEED;
                }
                break;
            case UP_TO_SPEED:
                _state = ShootingState.KICK_INTAKE;
                _timeout = System.currentTimeMillis() + 500;
                break;
            case KICK_INTAKE:
                // _cubeShooter.setWristSpeed(-0.2);
                // if (_timeout < System.currentTimeMillis()) {
                    _finished = true;
                // }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return _finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _cubeShooter.setShooterSpeed(0);
        _cubeShooter.setShooterRPS(0);
        _finished = false;
    }

    public enum ShootingState {
        INITIALIZE(0),
        REVVING_UP(1),
        UP_TO_SPEED(2),
        KICK_INTAKE(3);

        private final int _value;

        ShootingState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
