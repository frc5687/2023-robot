package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Arm extends OutliersSubsystem{
    private OutliersTalon _talon;
    private HallEffect _hall;

    public Arm(OutliersContainer container/* , HallEffect hall*/) {
        super(container);
        _talon = new OutliersTalon(0, Constants.Arm.CAN_BUS, "arm");
        _talon.configure(Constants.Arm.CONFIG);
        // _hall = hall;
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

    public double getArmAngleRadians() {
        return OutliersTalon.ticksToRadians(getEncoderTicks(), Constants.Arm.GEAR_RATIO);
    }

    public void updateDashboard() {}
}
