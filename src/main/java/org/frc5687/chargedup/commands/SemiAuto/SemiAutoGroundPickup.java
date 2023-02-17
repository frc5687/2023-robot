package org.frc5687.chargedup.commands.SemiAuto;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SemiAutoGroundPickup extends OutliersCommand{
    private EndEffector _endEffector;
    private Elevator _elevator;
    private Arm _arm;
    private OI _oi;
    public SemiAutoGroundPickup(
        Arm arm,
        EndEffector endEffector,
        Elevator elevator,
        OI oi
    ) {
        _endEffector = endEffector;
        _elevator = elevator;
        _arm = arm;
        _oi = oi;
    }

    @Override
    public void initialize() {
        super.initialize();
        if (_endEffector.getConeMode()) {
            (new SemiAutoGroundPickupCone(_arm, _endEffector, _elevator, _oi)).schedule();
        } else {
            (new SemiAutoGroundPickupCube(_arm, _endEffector, _elevator, _oi)).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
