package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.subsystems.Lights.AnimationType;
import org.frc5687.chargedup.OI;

public class DriveLights extends OutliersCommand {
   
   private final Lights _lights;
   private final EndEffector _endEffector;
   private final DriveTrain _driveTrain;
   private final OI _oi;

    public DriveLights(EndEffector endEffector, Lights lights, DriveTrain driveTrain, OI oi){
    _endEffector = endEffector;
    _lights = lights;
    _driveTrain = driveTrain;
    _oi = oi;
    addRequirements(lights);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _lights.switchAnimation(AnimationType.STATIC);
        _lights.setColor(Constants.CANdle.BLUE);
    }
    @Override
    public void execute() {
        super.execute();
        _lights.switchAnimation(AnimationType.STATIC);
        metric("isConeMode", _endEffector.getConeMode());
         if(_endEffector.getConeMode()){ 
            if(_oi.autoAim() && _driveTrain.isConeDetected()){
                _lights.switchAnimation(AnimationType.STROBE);
            } else{
                _lights.switchAnimation(AnimationType.STATIC);
            }
            _lights.setColor(Constants.CANdle.YELLOW);   
         } else { 
            if(_oi.autoAim() && _driveTrain.isCubeDetected()){
              _lights.switchAnimation(AnimationType.STROBE);
            } else{
              _lights.switchAnimation(AnimationType.STATIC);
            }
            _lights.setColor(Constants.CANdle.PURPLE);
         } 
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}
