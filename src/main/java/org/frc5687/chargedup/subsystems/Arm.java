package org.frc5687.chargedup.subsystems;

import static org.frc5687.chargedup.Constants.Arm.*;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;

public class Arm extends OutliersSubsystem {
    private final OutliersTalon _talon;
    private final DutyCycleEncoder _absAngleEncoder;
    private final Encoder _boreQuadEncoder;
    private double _relativeEncoderOffset;
    private boolean _hasZeroed;

    public Arm(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.ARM, Constants.Arm.CAN_BUS, "arm");
        _talon.configure(Constants.Arm.CONFIG);
        _talon.configureClosedLoop(CLOSED_LOOP_CONFIGURATION);

        _boreQuadEncoder = new Encoder(RobotMap.DIO.ARM_ENCODER_A, RobotMap.DIO.ARM_ENCODER_B, true);
        _boreQuadEncoder.setDistancePerPulse((Math.PI)/2048);

        _absAngleEncoder = new DutyCycleEncoder(RobotMap.DIO.ARM_ENCODER);
        _absAngleEncoder.setDistancePerRotation(Math.PI); // 2:1 from output to encoder
        _relativeEncoderOffset = getAbsoluteArmEncoderAngle();
        setEncoderRadians(getAbsoluteArmEncoderAngle());
        _hasZeroed = false;
    }

    public void periodic() {
        if (!_hasZeroed) {
            _relativeEncoderOffset = getAbsoluteArmEncoderAngle();
            setEncoderRadians(getAbsoluteArmEncoderAngle());
            _hasZeroed = true;
        }

        if (Math.abs(getRelativeEncoderAngle() - getArmAngleRadians()) > Units.degreesToRadians(2.5)) {
            error(" Arm encoder difference is larger than 2.5 degrees, skip has occurred! Setting Falcon encoder to Bore Encoder angle.");
            setEncoderRadians(getRelativeEncoderAngle());
        }
    }

    public void setArmSpeed(double speed) {
        _talon.setPercentOutput(speed);
    }

    public void setArmVoltage(double voltage) {
        // this normalized the 12 volts output between [-1, 1] ie. 6 volts / 12 volts = 50% speed.
        _talon.setVoltage(voltage);
    }

    /**
     * Set the motion magic angle to be the wanted angle in radians
     * @param angle in radians
     */
    public void setArmAngle(double angle) {
        _talon.setMotionMagic(OutliersTalon.radiansToRotations(angle, GEAR_RATIO));
    }

    public double getEncoderRotation() {
        return _talon.getPosition().getValue();
    }

    public double getAbsoluteArmEncoderAngle() {
        return _absAngleEncoder.getDistance();
    }

    public void zeroEncoder() {
        _talon.setRotorPosition(0.0);
    }

    public void setEncoderRadians(double angle) {
        _talon.setRotorPosition(OutliersTalon.radiansToRotations(angle, Constants.Arm.GEAR_RATIO));
    }

    public double getEncoderRotationsPerSec() {
        return _talon.getVelocity().getValue();
    }

    public double getArmAngleRadians() {
        //return getAbsoluteArmEncoderAngle();
//        return getRelativeEncoderAngle();
        return getArmAngleFalconRadians();
    }

    public double getArmAngleFalconRadians() {
        return OutliersTalon.rotationsToRadians(getEncoderRotation(), GEAR_RATIO);
    }

    public double getArmVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                OutliersTalon.rotationsPerSecToRPM(getEncoderRotationsPerSec(), GEAR_RATIO));
    }
    public double armFeedForward() {
        return ((ARM_LENGTH / 2.0) * (MOTOR_R * ARM_WEIGHT * 9.81) / (GEAR_RATIO * MOTOR_kT))
                * Math.cos(getArmAngleRadians() + (Math.PI / 2.0) - VERTICAL_ARM_ANGLE);
    }
    public double getRelativeEncoderAngle(){
        return _boreQuadEncoder.getDistance() + _relativeEncoderOffset;
    }

    public void updateDashboard() {
        metric("Angle", getArmAngleRadians());
        metric("Absolute Angle", getAbsoluteArmEncoderAngle());
        metric("Bore Encoder Angle", getRelativeEncoderAngle());
//        metric("Reference", _controlLoop.getNextR().get(0, 0));
    }
}
