package org.frc5687.chargedup.commands;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.subsystems.Lights.AnimationType;
import org.frc5687.chargedup.subsystems.Lights.LightsState;

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

        if (_lights.getState() != LightsState.OFF) {
            if (_endEffector.getConeMode()) {
                if (_oi.getSlowMode()) {
                    _lights.setState(LightsState.SLOWCONE);
                } else {
                    _lights.setState(LightsState.CONEMODE);
                }
            } else {
                if (_oi.getSlowMode()) {
                    _lights.setState(LightsState.SLOWCUBE);
                } else {
                    _lights.setState(LightsState.CUBEMODE);
                }
            }
        }

        if (DriverStation.isDisabled()) {
            _lights.switchAnimation(AnimationType.RAINBOW);
        } else {
            switch(_lights.getState()) {
                case OFF:
                    _lights.setColor(Constants.CANdle.OFF);
                    _lights.switchAnimation(AnimationType.STATIC);
                    break;
                case CONEMODE:
                    _lights.setColor(Constants.CANdle.YELLOW);
                    _lights.switchAnimation(AnimationType.STATIC);
                    break;
                case CUBEMODE:
                    _lights.setColor(Constants.CANdle.PURPLE);
                    _lights.switchAnimation(AnimationType.STATIC);
                    break;
                case SLOWCONE:
                    _lights.setColor(Constants.CANdle.YELLOW);
                    _lights.switchAnimation(AnimationType.SINGLE_FADE);
                    break;
                case SLOWCUBE:
                    _lights.setColor(Constants.CANdle.PURPLE);
                    _lights.switchAnimation(AnimationType.SINGLE_FADE);
                    break;
            }
        }
        // } else if (_endEffector.getConeMode()) {
        //     _lights.setColor(Constants.CANdle.YELLOW);
        //     if (_driveTrain.getSlowMode()) {
        //         _lights.switchAnimation(AnimationType.SINGLE_FADE);
        //     } else if (_oi.autoAim() && _driveTrain.isConeDetected()) {
        //         _lights.switchAnimation(AnimationType.STROBE);
        //     } else {
        //         _lights.switchAnimation(AnimationType.STATIC);
        //     }
        // } else {
        //     _lights.setColor(Constants.CANdle.PURPLE);
        //     if (_driveTrain.getSlowMode()) {
        //         _lights.switchAnimation(AnimationType.SINGLE_FADE);
        //     } else if (_oi.autoAim() && _driveTrain.isCubeDetected()) {
        //         _lights.switchAnimation(AnimationType.STROBE);
        //     } else {
        //         _lights.switchAnimation(AnimationType.STATIC);
        //     }
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
