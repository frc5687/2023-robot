package org.frc5687.chargedup.subsystems;

import edu.wpi.first.math.util.Units;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;

public class CubeShooter extends OutliersSubsystem {
    private final OutliersTalon _wrist;
    private final OutliersTalon _shooter;

    public CubeShooter(OutliersContainer container) {
        super(container);
        _wrist =
                new OutliersTalon(
                        RobotMap.CAN.TALONFX.CUBESHOOTER_WRIST,
                        Constants.CubeShooter.CAN_BUS,
                        "Cube Shooter Wrist");
        _shooter =
                new OutliersTalon(
                        RobotMap.CAN.TALONFX.CUBESHOOTER_SHOOTER,
                        Constants.CubeShooter.CAN_BUS,
                        "Cube Shooter Shooter");
        _wrist.configure(Constants.CubeShooter.WRIST_CONFIG);
        _wrist.configureClosedLoop(Constants.CubeShooter.CONTROLLER_CONFIG);
        _shooter.configure(Constants.CubeShooter.SHOOTER_CONFIG);
    }

    public void setWristSpeed(double speed) {
        _wrist.setPercentOutput(speed);
    }

    public void setShooterSpeed(double speed) {
        _shooter.setPercentOutput(speed);
    }

    public void setWristAngle(double angle) {
        _wrist.setMotionMagic(Units.radiansToRotations(angle));
    }

    public void setWristEncoderRotation(double rotation) {
        _wrist.setRotorPosition(rotation);
    }

    public void zeroWrist() {
        setWristEncoderRotation(0);
    }

    public double getWristCurrent() {
        return _wrist.getStatorCurrent().getValue();
    }

    public double getWristEncoderRotation() {
        return _wrist.getPosition().getValue();
    }

    public double getWristAngleRadians() {
        return Units.rotationsToRadians(getWristEncoderRotation());
    }

    public void updateDashboard() {
        metric("Wrist angle radians", getWristAngleRadians());
        metric("Wrist rotations", getWristEncoderRotation());
    }
}
