package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;

public class ExtendingArm extends OutliersSubsystem {
    private OutliersTalon _talon;
    private HallEffect _hall;

    public ExtendingArm(OutliersContainer container){
        super(container);
    } 

    public void updateDashboard() {}
}