package org.frc5687.chargedup.commands.SemiAuto;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SemiAutoPickOrPlace extends OutliersCommand {
    private EndEffector _endEffector;
    private Elevator _elevator;
    private Arm _arm;
    private OI _oi;

    public SemiAutoPickOrPlace(Arm arm, EndEffector endEffector, Elevator elevator, OI oi) {
        _endEffector = endEffector;
        _elevator = elevator;
        _arm = arm;
        _oi = oi;
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
