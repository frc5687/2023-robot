package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;

public class Elevator extends OutliersSubsystem {
    private OutliersTalon _talon;
    private final HallEffect _outHall;
    private final HallEffect _inHall;
    private boolean _hasZeroed;

    public Elevator(OutliersContainer container) {
        super(container);
        _talon =
                new OutliersTalon(
                        RobotMap.CAN.TALONFX.EXT_ARM, Constants.ExtendingArm.CAN_BUS, "ExtendingArm");
        _talon.configure(Constants.ExtendingArm.CONFIG);
        _talon.configureClosedLoop(Constants.ExtendingArm.CONTROLLER_CONFIG);

        _outHall = new HallEffect(RobotMap.DIO.OUT_EXT_HALL);
        _inHall = new HallEffect(RobotMap.DIO.IN_EXT_HALL);
        _hasZeroed = false;
    }

    @Override
    public void periodic() {
        super.periodic();
       /*
         if (_outHall.get() && !_hasZeroed) {
            _talon.setRotorPosition(Constants.ExtendingArm.OUT_HALL_ENCODER_ROTATIONS);
            _hasZeroed = true;
        } else {
            _hasZeroed = false;
        } */

        if (_inHall.get() && !_hasZeroed) {
            _talon.setRotorPosition(Constants.ExtendingArm.IN_HALL_RAD);
            _hasZeroed = true;
        } else if (!_inHall.get() && _hasZeroed) {
            _hasZeroed = false;
        }

    }

    public void setArmSpeed(double speed) {
        _talon.setPercentOutput(speed);
    }

    public void setExtArmLengthMeters(double distance) {
        // _talon.set(ControlMode.MotionMagic, (distance * Constants.ExtendingArm.TICKS_TO_METERS));
        _talon.setMotionMagic(distance * Constants.ExtendingArm.ROTATIONS_TO_METERS);
    }

    public double getExtArmMeters() {
        return getEncoderPositionRotations() / Constants.ExtendingArm.ROTATIONS_TO_METERS;
    }

    public void stopArm() {
        setArmSpeed(Constants.ExtendingArm.ZERO_ARM_SPEED);
    }

    public boolean getOutHall() {
        return _outHall.get();
    }

    public boolean getInHall() {
        return _inHall.get();
    }

    public void zeroEncoder() {
        _talon.setRotorPosition(Constants.ExtendingArm.ZERO_ENCODER);
    }

    public double getEncoderPositionRotations() {
        return _talon.getPosition().getValue();
    }

    public double getEncoderRotationRadians() {
        return OutliersTalon.rotationsToRadians(getEncoderPositionRotations(), 1.0);
    }

    public double getElevatorRotationsRadians() {
        return OutliersTalon.rotationsToRadians(
                getEncoderPositionRotations(), Constants.ExtendingArm.GEAR_RATIO);
    }

    public void updateDashboard() {
        metric("Encoder Position", getEncoderPositionRotations());
        metric("Encoder Radians", getEncoderRotationRadians());
        metric("Elevator Radians", getEncoderRotationRadians());
        metric("Elevator Length", getExtArmMeters());
        metric("Out Hall", getOutHall());
        metric("In Hall", getInHall());
        metric("Motor Output", _talon.get());
    }
}
