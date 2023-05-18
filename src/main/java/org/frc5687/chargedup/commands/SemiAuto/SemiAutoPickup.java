package org.frc5687.chargedup.commands.SemiAuto;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import org.frc5687.chargedup.Constants;

public class SemiAutoPickup extends OutliersCommand {
    private EndEffector _endEffector;
    private Elevator _elevator;
    private Arm _arm;
    private OI _oi;

    private IntakeState _state;
    private Setpoint _setpoint;
    private Setpoint _stowSetpoint;
    private boolean _isFinished;
    private boolean _isConeMode;
    private long _timeout;

    public SemiAutoPickup(Arm arm, EndEffector endEffector, Elevator elevator, OI oi) {
        _endEffector = endEffector;
        _elevator = elevator;
        _arm = arm;
        _oi = oi;
        _state = IntakeState.INITIALIZE;
        _setpoint = _endEffector.getConeMode() ? conePickupSetpoint : cubePickupSetpoint;
        _stowSetpoint = _endEffector.getConeMode() ? idleConeSetpoint : idleCubeSetpoint;
        _isFinished = false;
        _isConeMode = _endEffector.getConeMode();
        
        addRequirements(_arm, _endEffector, _elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        _state = IntakeState.INITIALIZE;
        _setpoint = _endEffector.getConeMode() ? conePickupSetpoint : cubePickupSetpoint;
        _stowSetpoint = _endEffector.getConeMode() ? idleConeSetpoint : idleCubeSetpoint;
        _isFinished = false;
        _isConeMode = _endEffector.getConeMode();
        error("Picking up");
    }

    @Override
    public void execute() {
        // error("Executinng semiauto pickup");
        updateDashboard();
        super.execute();
        if (_isConeMode != _endEffector.getConeMode()) {
            _setpoint = _endEffector.getConeMode() ? conePickupSetpoint : cubePickupSetpoint;
            _stowSetpoint = _endEffector.getConeMode() ? idleConeSetpoint : idleCubeSetpoint;
            _state = IntakeState.DEPLOY;
            error("Switching Modes and redeploying");
            _isConeMode = _endEffector.getConeMode();
        }
        switch (_state) {
            case INITIALIZE:
                _state = IntakeState.DEPLOY;
                break;
            case CLEAR_ELEVATOR:
                _elevator.setExtArmLengthMeters(Constants.Elevator.SHORT_ARM_DISTANCE);
                if (Math.abs(Constants.Elevator.SHORT_ARM_DISTANCE - _elevator.getExtArmMeters()) < Constants.Elevator.EXT_ARM_TOLERANCE) {
                    _state = IntakeState.DEPLOY;
                }
                break;
            case DEPLOY:
                _elevator.setExtArmLengthMeters(_setpoint.elevatorPosition);
                _endEffector.setWristSetpointRadians(_setpoint.wristAngle);
                _endEffector.setRollerSpeed(_setpoint.gripperSpeed);
                _endEffector.setWristSpeed(_endEffector.getWristControllerOutput());
                _arm.setArmAngle(_setpoint.armAngle);
                if (Math.abs(_setpoint.elevatorPosition - _elevator.getExtArmMeters()) < Constants.Elevator.EXT_ARM_TOLERANCE
                    && Math.abs(_setpoint.armAngle - _arm.getArmAngleRadians()) < Constants.Arm.ANGLE_TOLERANCE
                    && Math.abs(_setpoint.wristAngle - _endEffector.getWristAngleRadians()) < Constants.EndEffector.WRIST_TOLERENCE
                ) {
                    _state = IntakeState.WAITING_FOR_STOW;
                }
                break;
            case WAITING_FOR_STOW:
                _endEffector.setWristSpeed(_endEffector.getWristControllerOutput());
                if (_endEffector.isRollerStalled() || _oi.stow()) {
                    _timeout = System.currentTimeMillis() + Constants.EndEffector.WRIST_TIMEOUT;
                    _state = IntakeState.WAIT_WRIST;
                }
                break;
            case WAIT_WRIST:
                _endEffector.setWristSpeed(_endEffector.getWristControllerOutput());
                _endEffector.setRollerSpeed(_setpoint.gripperSpeed  * (_isConeMode ? 1.0 : 0.5));
                if (_timeout < System.currentTimeMillis()){
                    _state = IntakeState.STOW_WRIST;
                }
            break;
            case STOW_WRIST:
                _endEffector.setWristSetpointRadians(Constants.EndEffector.WRIST_SAFE_ANGLE);
                _endEffector.setRollerSpeed(_stowSetpoint.gripperSpeed);
                _endEffector.setWristSpeed(_endEffector.getWristControllerOutput());
                if (Math.abs(Constants.EndEffector.WRIST_SAFE_ANGLE - _endEffector.getWristAngleRadians()) < Constants.EndEffector.WRIST_TOLERENCE) {
                    _state = IntakeState.STOWING;
                }
                break;
            case STOWING:
                _elevator.setExtArmLengthMeters(_stowSetpoint.elevatorPosition);
                _endEffector.setWristSetpointRadians(_stowSetpoint.wristAngle);
                _endEffector.setRollerSpeed(_stowSetpoint.gripperSpeed);
                _endEffector.setWristSpeed(_endEffector.getWristControllerOutput());
                _arm.setArmAngle(_stowSetpoint.armAngle);
                if (Math.abs(_stowSetpoint.elevatorPosition - _elevator.getExtArmMeters()) < Constants.Elevator.EXT_ARM_TOLERANCE
                    && Math.abs(_stowSetpoint.armAngle - _arm.getArmAngleRadians()) < Constants.Arm.ANGLE_TOLERANCE
                    && Math.abs(_stowSetpoint.wristAngle - _endEffector.getWristAngleRadians()) < Constants.EndEffector.WRIST_TOLERENCE
                ) {
                    _state = IntakeState.FINISHED;
                }
                break;
            case FINISHED:
                _isFinished = true;
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return _isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _endEffector.setWristSpeed(0);
    }

    public enum IntakeState {
        INITIALIZE(0),
        CLEAR_ELEVATOR(1),
        DEPLOY(2),
        WAITING_FOR_STOW(3),
        WAIT_WRIST(4),
        STOW_WRIST(5),
        STOWING(6),
        FINISHED(7);

        private int _value;

        IntakeState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
    public void updateDashboard(){
        metric("Current State of Pickup: ", _state.toString());
    }
}
