package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;
import org.frc5687.chargedup.RobotMap;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;

public class ExtendingArm extends OutliersSubsystem {
    private OutliersTalon _talon;
    private HallEffect _topHall;
    private HallEffect _bottomHall;

    private final PIDController _extArmController;

    public ExtendingArm(OutliersContainer container){
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.EXT_ARM, Constants.ExtendingArm.CAN_BUS, "ExtendingArm");
        _talon.configure(Constants.ExtendingArm.CONFIG);
        _talon.configureClosedLoop(Constants.ExtendingArm.CONTROLLER_CONFIG);

        _extArmController = new PIDController(Constants.ExtendingArm.kP, Constants.ExtendingArm.kI, Constants.ExtendingArm.kD);
        _topHall = new HallEffect(RobotMap.DIO.TOP_EXT_HALL);
        _bottomHall = new HallEffect(RobotMap.DIO.BOTTOM_EXT_HALL);
    } 

    @Override
    public void periodic() {
        // super.periodic();
        // if (_topHall.get() && _talon.getLastSet() > 0){
            // _talon.set(ControlMode.PercentOutput, Constants.ExtendingArm.ZERO_ARM_SPEED);
            // _talon.setSelectedSensorPosition(OutliersTalon.radiansToTicks(Constants.ExtendingArm.TOP_HALL_RAD, Constants.ExtendingArm.GEAR_RATIO));
        // }

        // if (_bottomHall.get() && _talon.getLastSet() > 0){
            // _talon.set(ControlMode.PercentOutput, Constants.ExtendingArm.ZERO_ARM_SPEED);
            // _talon.setSelectedSensorPosition(OutliersTalon.radiansToTicks(Constants.ExtendingArm.BOTTOM_HALL_RAD, Constants.ExtendingArm.GEAR_RATIO));
        // }
    }

    public void setArmSpeed(double speed) {
        _talon.set(ControlMode.PercentOutput, speed);
    }

    public void setExtArmLengthMeters(double distance){
        _talon.set(ControlMode.MotionMagic, (distance * Constants.ExtendingArm.TICKS_TO_METERS));
    }

    public double getExtArmMeters(){
        return getEncoderTicks() / Constants.ExtendingArm.TICKS_TO_METERS;
    }

    public void stopArm(){
        setArmSpeed(Constants.ExtendingArm.ZERO_ARM_SPEED);
    }

    public boolean getTopHall() {
        return _topHall.get();
    }

    public boolean getBottomHall() {
        return _bottomHall.get();
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

    public void updateDashboard() {
        metric("Encoder Ticks", getEncoderTicks());
        metric("Arm Extended Length", getExtArmMeters());
        metric("Set Point", _talon.getLastSet());
    }
}