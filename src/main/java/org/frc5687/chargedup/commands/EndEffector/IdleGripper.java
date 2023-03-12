package org.frc5687.chargedup.commands.EndEffector;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.EndEffector;

public class IdleGripper extends OutliersCommand {
    private EndEffector _endEffector;

    private OI _oi;

    public IdleGripper(EndEffector endEffector, OI oi) {
        _endEffector = endEffector;
        _oi = oi;
        addRequirements(_endEffector);
    }

    @Override
    public void execute() {
        super.execute();
        if (_endEffector.getConeMode()) {
            //            if (_oi.releaseRoller()) {
            //                _endEffector.setRollerSpeed(Constants.EndEffector.PLACE_CONE_ROLLER_SPEED);
            //            } else {
            _endEffector.setRollerSpeed(Constants.EndEffector.ROLLER_CONE_IDLE_SPEED);
            //            }
        } else {
            //            if (_oi.releaseRoller()) {
            //                _endEffector.setRollerSpeed(Constants.EndEffector.PLACE_CUBE_ROLLER_SPEED);
            //            } else {
            _endEffector.setRollerSpeed(Constants.EndEffector.ROLLER_CUBE_IDLE_SPEED);
            //            }
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
