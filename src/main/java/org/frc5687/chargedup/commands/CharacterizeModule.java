package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.subsystems.DiffSwerveModule;
import org.frc5687.chargedup.subsystems.DriveTrain;

/**
 * This is a command to characterize each swerve module finding the current to
 * overcome the friction of the motor and static friction of the module
 */
public class CharacterizeModule extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private CharacterizationState _state;
    private boolean _finished = false;

    public CharacterizeModule(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        addRequirements(_driveTrain);
    }


    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setModulesToCharacterization();
        _finished = false;
        _driveTrain.resetModuleEncoders();
        _state = CharacterizationState.INITIALIZE;
    }

    @Override
    public void execute() {
        super.execute();
        switch (_state) {
            case INITIALIZE:
                error(" Starting module characterization");
                _state = CharacterizationState.CHARACTERIZE_NORTH_EAST;
                break;
            case CHARACTERIZE_NORTH_EAST:
                _driveTrain.characterizeModules(3);
                if (_driveTrain.getCharacterizationState(3) == DiffSwerveModule.CharacterizeModule.FINISHED) {
                    _state = CharacterizationState.CHARACTERIZE_NORTH_WEST;
                }
                break;
            case CHARACTERIZE_NORTH_WEST:
                _driveTrain.characterizeModules(0);
                if (_driveTrain.getCharacterizationState(0) == DiffSwerveModule.CharacterizeModule.FINISHED) {
                    _state = CharacterizationState.CHARACTERIZE_SOUTH_EAST;
                }
                break;
            case CHARACTERIZE_SOUTH_EAST:
                _driveTrain.characterizeModules(2);
                if (_driveTrain.getCharacterizationState(2) == DiffSwerveModule.CharacterizeModule.FINISHED) {
                    _state = CharacterizationState.CHARACTERIZE_SOUTH_WEST;
                }
                break;
            case CHARACTERIZE_SOUTH_WEST:
                _driveTrain.characterizeModules(1);
                if (_driveTrain.getCharacterizationState(1) == DiffSwerveModule.CharacterizeModule.FINISHED) {
                    _state = CharacterizationState.FINISHED;
                }
                break;
            case FINISHED:
                _finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return _finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _driveTrain.startModules();
    }

    public enum CharacterizationState {
        INITIALIZE(0),
        CHARACTERIZE_NORTH_EAST(1),
        CHARACTERIZE_NORTH_WEST(2),
        CHARACTERIZE_SOUTH_EAST(3),
        CHARACTERIZE_SOUTH_WEST(4),
        FINISHED(5);

        private int _value;

        CharacterizationState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}

