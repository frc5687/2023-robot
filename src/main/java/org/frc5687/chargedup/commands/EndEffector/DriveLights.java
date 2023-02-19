package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.subsystems.Lights.AnimationType;

import edu.wpi.first.wpilibj.DriverStation;

import org.frc5687.chargedup.OI;

public class DriveLights extends OutliersCommand {
   
   private Lights _lights;
   private EndEffector _endEffector;
   private DriveTrain _driveTrain;
   private OI _oi;

    public DriveLights(/*EndEffector endEffector, */ Lights lights,/*  DriveTrain driveTrain, */ OI oi){
    // _endEffector = endEffector;
    _lights = lights;
    // _driveTrain = driveTrain;
    _oi = oi;
    addRequirements(_lights);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        // _lights.setColor(Constants.CANdle.PURPLE);
        // _lights.switchAnimation(AnimationType.LARSON);
    }
    @Override
    public void execute() {
        super.execute();
        _lights.setColor(Constants.CANdle.RED);
        _lights.switchAnimation(AnimationType.TWINKLE);
        if (DriverStation.isDisabled()) {
            _lights.switchAnimation(AnimationType.RAINBOW);
        } else {
            if(/*_endEffector.getConeMode()*/ true){ 
                _lights.setColor(Constants.CANdle.YELLOW);
                if(_oi.autoAim() /*&& _driveTrain.isConeDetected()*/){
                    _lights.switchAnimation(AnimationType.STROBE);
                } else {
                    _lights.switchAnimation(AnimationType.STATIC);
                }   
            } else { 
                _lights.setColor(Constants.CANdle.PURPLE);
                if(_oi.autoAim() /*&& _driveTrain.isCubeDetected()*/){
                    _lights.switchAnimation(AnimationType.STROBE);
                } else {
                    _lights.switchAnimation(AnimationType.STATIC);
                }    
            }
        }
        
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}
