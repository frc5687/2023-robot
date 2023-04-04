package org.frc5687.chargedup.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.CubeShooter;
import org.frc5687.chargedup.subsystems.DriveTrain;

public class LevelingAndIntake extends OutliersCommand {
    private final DriveTrain _drivetrain;
    private final CubeShooter _cubeShooter;
    private LevelingPickupState _state;
    private final PIDController _pitchController;
    private boolean _finished;
    private long _timeout;

    public LevelingAndIntake(DriveTrain driveTrain, CubeShooter shooter) {
        _drivetrain = driveTrain;
        _cubeShooter = shooter;
        _pitchController = new PIDController(Constants.DriveTrain.AUTO_LEVEL_KP, Constants.DriveTrain.AUTO_LEVEL_KI, Constants.DriveTrain.AUTO_LEVEL_KD); // kp = 3.0 ki = 0.0 kd  = 0.5
        _finished = false;
        _state = LevelingPickupState.INITIAL;
        _timeout = System.currentTimeMillis();
        addRequirements(_drivetrain, _cubeShooter);
    }

    @Override
    public void initialize() {
        error(" Starting Drive");
        _state = LevelingPickupState.INITIAL;
    }

    @Override
    public void execute() {
        metric("Leveling State", String.valueOf(_state));
        switch (_state) {
            case INITIAL:
                _state = LevelingPickupState.LOOKING_FOR_PITCH;
                break;
            case LOOKING_FOR_PITCH:
                _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                Constants.DriveTrain.DRIVING_UP_RAMP_SPEEDS_VX,
                                0.0,
                                0.0,
                                _drivetrain.getHeading()));
                if (Math.abs(_drivetrain.getPitch()) > Constants.DriveTrain.PITCH_LOOKING_ANGLE) {
                    _state = LevelingPickupState.LEVELING;
                }
                break;
            case LEVELING:
                _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                Constants.DriveTrain.DRIVING_UP_RAMP_SPEEDS_VX,
                                0.0,
                                0.0,
                                _drivetrain.getHeading()));
                if (Math.abs(_drivetrain.getPitch()) < Constants.DriveTrain.PITCH_LEVELED_ANGLE) {
                    _drivetrain.setVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.0, 0.0, _drivetrain.getHeading()));
                    _state = LevelingPickupState.LEVELED;
                }
                break;
            case LEVELED:
//                _state = LevelingPickupState.EXIT_LEVELING;
                _timeout = System.currentTimeMillis() + 1300;
                _state = LevelingPickupState.DRIVE_FORWARD_AND_GRAB;
                break;
            case EXIT_LEVELING:
                _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                Constants.DriveTrain.DRIVING_DOWN_RAMP_SPEEDS_VX,
                                0.0,
                                0.0,
                                _drivetrain.getHeading()));
                if (_drivetrain.getPitch() > Constants.DriveTrain.PITCH_LOOKING_ANGLE) {
                    _timeout = System.currentTimeMillis() + 1500;
                    _state = LevelingPickupState.DRIVE_FORWARD_AND_GRAB;
                }
                break;
            case DRIVE_FORWARD_AND_GRAB:
                if (!_cubeShooter.isCubeDetected()) {
                    _cubeShooter.setWristAngle(Constants.CubeShooter.INTAKE_ANGLE);
                    _cubeShooter.setShooterSpeed(-0.4);
                } else {
                    _cubeShooter.setShooterSpeed(0.0);
                    _cubeShooter.setWristAngle(Constants.CubeShooter.IDLE_ANGLE);
                }
                if (_timeout > System.currentTimeMillis()) {
                    _drivetrain.setVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    2.0, // this is the intaking pickup speed
                                    0.0,
                                    0.0,
                                    _drivetrain.getHeading()));
                } else {
                    _state = LevelingPickupState.LOOKING_FOR_PITCH_AGAIN;
                    _cubeShooter.setShooterSpeed(0.0);
                    _cubeShooter.setWristAngle(Constants.CubeShooter.IDLE_ANGLE);
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
                    _state = LevelingPickupState.LEVELING_AGAIN;
                }
                break;
            case LEVELING_AGAIN:
                double pow = _pitchController.calculate(_drivetrain.getPitch());
                _drivetrain.setVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(pow, 0.0, 0.0, _drivetrain.getHeading()));
                if (Math.abs(_drivetrain.getPitch()) < Constants.DriveTrain.PITCH_LEVELED_ANGLE) {
                    _drivetrain.setVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.0, 0.0, _drivetrain.getHeading()));
                    _state = LevelingPickupState.LEVELED_AGAIN;
                }
                break;
            case LEVELED_AGAIN:
                _finished = true;
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return _finished;
    }

    public enum LevelingPickupState {
        INITIAL(0),
        LOOKING_FOR_PITCH(1),
        LEVELING(2),
        LEVELED(3),
        EXIT_LEVELING(4),
        DRIVE_FORWARD_AND_GRAB(5),
        LOOKING_FOR_PITCH_AGAIN(6),
        LEVELING_AGAIN(7),
        LEVELED_AGAIN(8);

        private final int _value;

        LevelingPickupState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
