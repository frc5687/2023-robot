package org.frc5687.chargedup.commands;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.subsystems.Lights.AnimationType;

public class DriveLights extends OutliersCommand {

    private Lights _lights;
    private EndEffector _endEffector;
    private DriveTrain _driveTrain;
    private OI _oi;

    public DriveLights(EndEffector endEffector, Lights lights, DriveTrain driveTrain, OI oi) {
        _endEffector = endEffector;
        _lights = lights;
        _driveTrain = driveTrain;
        _oi = oi;
        addRequirements(_lights);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();

        if (DriverStation.isDisabled()) {
            _lights.switchAnimation(AnimationType.RAINBOW, 0);
            _lights.switchAnimation(AnimationType.RAINBOW, 1);
        } else if (_endEffector.getConeMode()) {
            _lights.setColor(Constants.CANdle.YELLOW, 0);
            _lights.setColor(Constants.CANdle.YELLOW, 1);
            switch (_driveTrain.getMode()) {
                case VISION:
                    if (_driveTrain.isConeDetected()) {
                        _lights.switchAnimation(AnimationType.STROBE, 0);
                        _lights.switchAnimation(AnimationType.STROBE, 1);

                    } else {
                        _lights.switchAnimation(AnimationType.STATIC, 0);
                        _lights.switchAnimation(AnimationType.STATIC, 1);

                    }
                    break;
                case SLOW:
                    _lights.switchAnimation(AnimationType.SINGLE_FADE, 0);
                    _lights.switchAnimation(AnimationType.SINGLE_FADE, 1);
                    break;
                case NORMAL:
                    _lights.switchAnimation(AnimationType.STATIC, 0);
                    _lights.switchAnimation(AnimationType.STATIC, 1);
                    break;
                default:
                    _lights.switchAnimation(AnimationType.STATIC, 0);
                    _lights.switchAnimation(AnimationType.STATIC, 1);
                    break;
            }
        } else {
            _lights.setColor(Constants.CANdle.PURPLE, 0);
            _lights.setColor(Constants.CANdle.PURPLE, 1);

            switch (_driveTrain.getMode()) {
                case VISION:
                    if (_driveTrain.isCubeDetected()) {
                        _lights.switchAnimation(AnimationType.STROBE, 0);
                        _lights.switchAnimation(AnimationType.STROBE, 1);

                    } else {
                        _lights.switchAnimation(AnimationType.STATIC, 0);
                        _lights.switchAnimation(AnimationType.STATIC, 1);

                    }
                    break;
                case SLOW:
                    _lights.switchAnimation(AnimationType.SINGLE_FADE, 0);
                    _lights.switchAnimation(AnimationType.SINGLE_FADE, 1);
                    break;
                case NORMAL:
                    _lights.switchAnimation(AnimationType.STATIC, 0);
                    _lights.switchAnimation(AnimationType.STATIC, 1);
                    break;
                default:
                    _lights.switchAnimation(AnimationType.STATIC, 0);
                    _lights.switchAnimation(AnimationType.STATIC, 1);
                    break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
