package org.frc5687.chargedup.commands.CubeShooter;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;

public class Shoot extends OutliersCommand {

    private long _timeout;
    private ShootingState _state;
    private final CubeShooter _cubeShooter;
    private final OI _oi;
    private final double _speed;
    private final double _angle;
    private boolean _finished;

    public Shoot(CubeShooter shooter, double speed, double angle, OI oi) {
        _cubeShooter = shooter;
        _speed = speed;
        _angle = angle;
        _state = ShootingState.INITIALIZE;
        _finished = false;
        _oi = oi;
        addRequirements(_cubeShooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        _state = ShootingState.INITIALIZE;
        _finished = false;
        _cubeShooter.setWristAngle(_angle);
    }

    @Override
    public void execute() {
        switch (_state) {
            case INITIALIZE:
                //                _cubeShooter.setShooterSpeed(-0.5);
                metric("wrist angle", _cubeShooter.getWristAngleRadians());
                metric("Wanted Angle", _angle);
                if (Math.abs(_cubeShooter.getWristAngleRadians() - _angle)
                                < Constants.CubeShooter.WRIST_ANGLE_TOLERANCE
                        && _oi.releaseRoller()) {
                    _timeout = System.currentTimeMillis() + 750;
                    _state = ShootingState.WRIST_AT_ANGLE;
                }
                // _state = ShootingState.REVVING_UP;
                break;
            case WRIST_AT_ANGLE:
                _cubeShooter.setShooterSpeed(_speed);
                if (_timeout < System.currentTimeMillis()) {
                    _finished = true;
                }
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
        WRIST_AT_ANGLE(1);

        private final int _value;

        ShootingState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
