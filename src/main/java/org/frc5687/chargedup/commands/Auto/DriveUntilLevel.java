package org.frc5687.chargedup.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.util.FieldConstants;

public class DriveUntilLevel extends OutliersCommand {

    private final DriveTrain _drivetrain;
    private LevelingState _state;
    private final PIDController _pitchController;
    private boolean _finished;
    private long _timeout;

    public DriveUntilLevel(DriveTrain driveTrain) {
        _drivetrain = driveTrain;
        if (FieldConstants.isRealField){ // If we are on the real field, use constants measured from Greater Boston
        _pitchController = new PIDController(Constants.DriveTrain.AUTO_LEVEL_KP, Constants.DriveTrain.AUTO_LEVEL_KI, Constants.DriveTrain.AUTO_LEVEL_KD); // kp = 3.0 ki = 0.0 kd  = 0.5
        } else { // Otherwise use constants measured from the practice field
        _pitchController = new PIDController(Constants.DriveTrain.PF_AUTO_LEVEL_KP, Constants.DriveTrain.PF_AUTO_LEVEL_KI, Constants.DriveTrain.PF_AUTO_LEVEL_KD); // kp = 3.0, ki = 0.0, kd = 0.5
        }
        _finished = false;
        _state = LevelingState.INITIAL;
        _timeout = System.currentTimeMillis();
        addRequirements(_drivetrain);
    }

    @Override
    public void initialize() {
        error(" Starting Drive");
        _state = LevelingState.INITIAL;
    }

    @Override
    public void execute() {
        metric("Leveling State", String.valueOf(_state));
        switch (_state) {
            case INITIAL:
                _state = LevelingState.LOOKING_FOR_PITCH;
                break;
            case LOOKING_FOR_PITCH:
                _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                Constants.DriveTrain.DRIVING_UP_RAMP_SPEEDS_VX,
                                0.0,
                                0.0,
                                _drivetrain.getHeading()));
                if (Math.abs(_drivetrain.getPitch()) > Constants.DriveTrain.PITCH_LOOKING_ANGLE) {
                    _state = LevelingState.LEVELING;
                }
                break;
            case LEVELING:
                double pow = _pitchController.calculate(_drivetrain.getPitch());
                _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(pow, 0.0, 0.0, _drivetrain.getHeading()));
                if (Math.abs(_drivetrain.getPitch()) < Constants.DriveTrain.PITCH_LEVELED_ANGLE) {
                    _drivetrain.setVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.0, 0.0, _drivetrain.getHeading()));
                    _state = LevelingState.LEVELED;
                }
                break;
            case LEVELED:
                _state = LevelingState.EXIT_LEVELING;
                break;
            case EXIT_LEVELING:
                _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                Constants.DriveTrain.DRIVING_DOWN_RAMP_SPEEDS_VX,
                                0.0,
                                0.0,
                                _drivetrain.getHeading()));
                if (_drivetrain.getPitch() > Constants.DriveTrain.PITCH_LOOKING_ANGLE) {
                    _timeout = System.currentTimeMillis() + 1000;
                    _state = LevelingState.DRIVE_FORWARD;
                }
                break;
            case DRIVE_FORWARD:
                if (_timeout > System.currentTimeMillis()) {
                    _drivetrain.setVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    Constants.DriveTrain.DRIVING_DOWN_RAMP_SPEEDS_VX,
                                    0.0,
                                    0.0,
                                    _drivetrain.getHeading()));
                } else {
                    _state = LevelingState.LOOKING_FOR_PITCH_AGAIN;
                }
                break;
            case LOOKING_FOR_PITCH_AGAIN:
                _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                -Constants.DriveTrain.DRIVING_UP_RAMP_SPEEDS_VX,
                                0.0,
                                0.0,
                                _drivetrain.getHeading()));
                if (Math.abs(_drivetrain.getPitch()) > Constants.DriveTrain.PITCH_LOOKING_ANGLE) {
                    _state = LevelingState.LEVELING_AGAIN;
                }
                break;
            case LEVELING_AGAIN:
                pow = _pitchController.calculate(_drivetrain.getPitch());
                _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(pow, 0.0, 0.0, _drivetrain.getHeading()));
                if (Math.abs(_drivetrain.getPitch()) < Constants.DriveTrain.PITCH_LEVELED_ANGLE) {
                    _drivetrain.setVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.0, 0.0, _drivetrain.getHeading()));
                }
                break;
            case LEVELED_AGAIN:
//                _finished = true;
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return _finished;
    }

    public enum LevelingState {
        INITIAL(0),
        LOOKING_FOR_PITCH(1),
        LEVELING(2),
        LEVELED(3),
        EXIT_LEVELING(4),
        DRIVE_FORWARD(5),
        LOOKING_FOR_PITCH_AGAIN(6),
        LEVELING_AGAIN(7),
        LEVELED_AGAIN(8);

        private final int _value;

        LevelingState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
