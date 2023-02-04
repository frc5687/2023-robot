package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;
import org.frc5687.chargedup.RobotMap;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;

public class Elevator extends OutliersSubsystem {
    private OutliersTalon _talon;
    private HallEffect _outHall;
    private HallEffect _inHall;

    public Elevator(OutliersContainer container){
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.EXT_ARM, Constants.ExtendingArm.CAN_BUS, "ExtendingArm");
        _talon.configure(Constants.ExtendingArm.CONFIG);
        _talon.configureClosedLoop(Constants.ExtendingArm.CONTROLLER_CONFIG);

        _outHall = new HallEffect(RobotMap.DIO.OUT_EXT_HALL);
        _inHall = new HallEffect(RobotMap.DIO.IN_EXT_HALL);
    } 

    @Override
    public void periodic() {
         super.periodic();
         if (_outHall.get()){
            //  _talon.set(ControlMode.PercentOutput, Constants.ExtendingArm.ZERO_ARM_SPEED);
             _talon.setSelectedSensorPosition(OutliersTalon.radiansToTicks(Constants.ExtendingArm.OUT_HALL_RAD, Constants.ExtendingArm.GEAR_RATIO));
         }

         if (_inHall.get()){
            //  _talon.set(ControlMode.PercentOutput, Constants.ExtendingArm.ZERO_ARM_SPEED);
             _talon.setSelectedSensorPosition(OutliersTalon.radiansToTicks(Constants.ExtendingArm.IN_HALL_RAD, Constants.ExtendingArm.GEAR_RATIO));
         }
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

    public boolean getOutHall() {
        return _outHall.get();
    }

    public boolean getInHall() {
        return _inHall.get();
    }

    public void zeroEncoder(){
        _talon.setSelectedSensorPosition(Constants.ExtendingArm.ZERO_ENCODER);
    }

    public double getEncoderTicks() {
        return _talon.getSelectedSensorPosition();
    }

    public double getEncoderRotationRadians() {
        return _talon.getSelectedSensorPosition() * ((2.0 * Math.PI)/ (2048 * Constants.ExtendingArm.GEAR_RATIO));
    }
    public void updateDashboard() {
        metric("Encoder Ticks", getEncoderTicks());
        metric("Encoder Radians", getEncoderRotationRadians());
        metric("Arm Extended Length", getExtArmMeters());
        metric("Out Hall", getOutHall());
        metric("In Hall", getInHall());
        metric("Motor Output", _talon.getMotorOutputPercent());
    }
}