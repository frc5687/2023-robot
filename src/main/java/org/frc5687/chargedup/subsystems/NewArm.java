package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class NewArm extends OutliersSubsystem{
    public OutliersTalon _talon;
    public DutyCycleEncoder _absAngleEncoder;
    
    public NewArm(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.ARM, "rio", "Arm");
        _talon.configureClosedLoop(Constants.Arm.CLOSED_LOOP_CONFIGURATION);
        _absAngleEncoder = new DutyCycleEncoder(RobotMap.DIO.ARM_ENCODER);
        _absAngleEncoder.setDistancePerRotation(Math.PI); // 2:1 from output to encoder
    }

    public double getArmAngleRadians() {
        return _absAngleEncoder.getDistance();
    }

    public void setArmAngle(double angle) {
        _talon.setMotionMagic(OutliersTalon.radiansToRotations(angle, Constants.Arm.GEAR_RATIO));
    }

    @Override
    public void updateDashboard() {
        metric("Arm Angle Radians", getArmAngleRadians());
    }
}
