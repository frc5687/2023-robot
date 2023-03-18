package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class IdleGripper extends OutliersCommand {
    private EndEffector _endEffector;
  


    public IdleGripper(EndEffector endEffector, OI oi) {
        _endEffector = endEffector;
      /*   _timeoutIn = (System.currentTimeMillis() + 400);
        _timeoutOut = (System.currentTimeMillis() + 150); */
        addRequirements(_endEffector);
    }

    @Override
    public void execute() {
        super.execute();
        if (_endEffector.getConeMode()) {
            _endEffector.setRollerSpeed(Constants.EndEffector.ROLLER_CONE_IDLE_SPEED);
            /* if (_timeoutOut > System.currentTimeMillis()) {
                _endEffector.setRollerSpeed(Constants.EndEffector.ROLLER_CONE_IDLE_SPEED);
            } else {
                _timeoutIn = (System.currentTimeMillis() + 400);
            } if (_timeoutIn < System.currentTimeMillis()){
                _endEffector.setRollerSpeed(-Constants.EndEffector.ROLLER_CONE_IDLE_SPEED);
            } else {
                _timeoutOut = (System.currentTimeMillis() + 150);
            } */

            

            

                          
        } else {
            _endEffector.setRollerSpeed(Constants.EndEffector.ROLLER_CUBE_IDLE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
