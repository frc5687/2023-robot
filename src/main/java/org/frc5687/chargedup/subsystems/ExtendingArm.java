package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;


import com.ctre.phoenix.motorcontrol.ControlMode;

public class ExtendingArm extends OutliersSubsystem {
    private OutliersTalon _talon;
    private HallEffect _hall;

    public ExtendingArm(OutliersContainer container){
        super(container);
        _talon = new OutliersTalon talon(0, Constants.ExtendingArm.CAN_BUS, "ExtendingArm");
        talon.configure(Constants.ExtendingArm.CONFIG);
    } 

    public void setArmSpeed(double speed) {
        _talon.set(ControlMode.PercentOutput, speed);
    }

    public boolean getHall() {
        return _hall.get();
    }

    public double getEncoderTicks() {
        return _talon.getSelectedSensorPosition();
    }

    public void updateDashboard() {}
}