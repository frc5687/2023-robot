package org.frc5687.chargedup.commands.Auto;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.commands.Auto.DriveUntilLevel.LevelingState;
import org.frc5687.chargedup.subsystems.DriveTrain;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class QuickLevel extends OutliersCommand {
    
    private final DriveTrain _drivetrain;
    private LevelingState _state;
    private final PIDController _pitchController;
    private boolean _finished;

    public QuickLevel(DriveTrain drivetrain) {
        _drivetrain = drivetrain;
        _pitchController = new PIDController(Constants.DriveTrain.AUTO_LEVEL_KP, Constants.DriveTrain.AUTO_LEVEL_KI, Constants.DriveTrain.AUTO_LEVEL_KD);
        _finished = false;
        _state = LevelingState.INITIAL;
        addRequirements(_drivetrain);
    }

    @Override
    public void initialize() {
        error("Starting to Level");
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
                        -Constants.DriveTrain.DRIVING_UP_RAMP_SPEEDS_VX,
                        0.0,
                        0.0,
                        _drivetrain.getHeading()));
                if (Math.abs(_drivetrain.getPitch()) > Constants.DriveTrain.PITCH_LOOKING_ANGLE) {
                    _state = LevelingState.LEVELING;
                }   
                break;
            case LEVELING:
                double pow  = _pitchController.calculate(_drivetrain.getPitch());
                _drivetrain.setVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(pow, 0.0, 0.0, _drivetrain.getHeading()));
                if (Math.abs(_drivetrain.getPitch()) < Constants.DriveTrain.PITCH_LEVELED_ANGLE) {
                    _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, _drivetrain.getHeading()));    
                    _state = LevelingState.LEVELED;
                }
                break;
            case LEVELED:
                 _finished = true;
            default:
                error("Uh Oh, default");
                break;
                        
        }   
    }
    @Override
    public boolean isFinished() {
        return _finished;
    }

}
