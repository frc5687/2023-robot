package org.frc5687.chargedup.subsystems;

import javax.swing.text.Position;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;
import org.frc5687.chargedup.RobotMap;


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
             //_talon.setSelectedSensorPosition(OutliersTalon.radiansToTicks(Constants.ExtendingArm.OUT_HALL_RAD, Constants.ExtendingArm.GEAR_RATIO));
            _talon.setRotorPosition(Constants.ExtendingArm.OUT_HALL_RAD / (Math.PI * 2));
         }

         if (_inHall.get()){
            //  _talon.set(ControlMode.PercentOutput, Constants.ExtendingArm.ZERO_ARM_SPEED);
            //  _talon.setSelectedSensorPosition(OutliersTalon.radiansToTicks(Constants.ExtendingArm.IN_HALL_RAD, Constants.ExtendingArm.GEAR_RATIO));
            _talon.setRotorPosition(Constants.ExtendingArm.IN_HALL_RAD / (Math.PI * 2));
        }
    }

    public void setArmSpeed(double speed) {
        _talon.setPercentOutput(speed);
    }

    public void setExtArmLengthMeters(double distance){
        //_talon.set(ControlMode.MotionMagic, (distance * Constants.ExtendingArm.TICKS_TO_METERS));
        _talon.setMotionMagic(distance);
    }

    public double getExtArmMeters(){
        return getEncoderPosition() / Constants.ExtendingArm.TICKS_TO_METERS;
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
        _talon.setRotorPosition(Constants.ExtendingArm.ZERO_ENCODER);
    }

    public double getEncoderPosition() {
        return _talon.getPosition().getValue();
    }

    public double getEncoderRotationRadians() {
        return getEncoderPosition() * ((2.0 * Math.PI)/ (Constants.ExtendingArm.GEAR_RATIO));
    }
    public void updateDashboard() {
        metric("Encoder Position", getEncoderPosition());
        metric("Encoder Radians", getEncoderRotationRadians());
        metric("Arm Extended Length", getExtArmMeters());
        metric("Out Hall", getOutHall());
        metric("In Hall", getInHall());
        metric("Motor Output", _talon.get());
    }
}