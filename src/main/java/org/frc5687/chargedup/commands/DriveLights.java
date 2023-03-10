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
            _lights.switchAnimation(AnimationType.RAINBOW);
        } else if (_endEffector.getConeMode()) {
            _lights.setColor(Constants.CANdle.YELLOW);
            switch (_driveTrain.getMode()) {
                case VISION:
                    if (_driveTrain.isConeDetected()) {
                        _lights.switchAnimation(AnimationType.STROBE);
                    } else {
                        _lights.switchAnimation(AnimationType.STATIC);
                    }
                    break;
                case SLOW:
                    _lights.switchAnimation(AnimationType.SINGLE_FADE);
                    break;
                case NORMAL:
                    _lights.switchAnimation(AnimationType.STATIC);
                    break;
                default:
                    _lights.switchAnimation(AnimationType.STATIC);
                    break;
            }
        } else {
            _lights.setColor(Constants.CANdle.PURPLE);
            switch (_driveTrain.getMode()) {
                case VISION:
                    if (_driveTrain.isCubeDetected()) {
                        _lights.switchAnimation(AnimationType.STROBE);
                    } else {
                        _lights.switchAnimation(AnimationType.STATIC);
                    }
                    break;
                case SLOW:
                    _lights.switchAnimation(AnimationType.SINGLE_FADE);
                    break;
                case NORMAL:
                    _lights.switchAnimation(AnimationType.STATIC);
                    break;
                default:
                    _lights.switchAnimation(AnimationType.STATIC);
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
