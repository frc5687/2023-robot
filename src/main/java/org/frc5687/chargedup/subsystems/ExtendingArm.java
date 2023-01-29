package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;

public class ExtendingArm extends OutliersSubsystem {
    private OutliersTalon _talon;
    private HallEffect _hall;

    private final PIDController _extArmController;

    public ExtendingArm(OutliersContainer container){
        super(container);
        _talon = new OutliersTalon(0, Constants.ExtendingArm.CAN_BUS, "ExtendingArm");
        _talon.configure(Constants.ExtendingArm.CONFIG);

        _extArmController = new PIDController(Constants.ExtendingArm.kP, Constants.ExtendingArm.kI, Constants.ExtendingArm.kD);
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

    public double getEncoderRotationRadians() {
        return _talon.getSelectedSensorPosition() * ((2.0 * Math.PI)/ Constants.ExtendingArm.GEAR_RATIO);
    }

    public void setExtArmSetpointDistance(double distance){
        _extArmController.setSetpoint(distance);
    }

    public double getExtArmControllerOutput(){
        return _extArmController.calculate(getEncoderRotationRadians());
    }

    public void updateDashboard() {}
}