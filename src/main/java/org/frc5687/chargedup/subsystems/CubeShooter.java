package org.frc5687.chargedup.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.Nodes;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.ProximitySensor;

public class CubeShooter extends OutliersSubsystem {
    private final OutliersTalon _wrist;
    private final OutliersTalon _shooter;
    private final DutyCycleEncoder _wristEncoder;
    private final ProximitySensor _proximitySensor;

    public CubeShooter(OutliersContainer container) {
        super(container);
        _wristEncoder = new DutyCycleEncoder(RobotMap.DIO.ENCODER_CUBESHOOTER_WRIST);
        _proximitySensor = new ProximitySensor(RobotMap.DIO.CUBESHOOTER_PROXIMITY);
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
        _wrist.configureClosedLoop(Constants.CubeShooter.CONTROLLER_CONFIG_WRIST);
        _wrist.setRotorPosition(getBoreEncoderRotation() * Constants.CubeShooter.GEAR_RATIO);

        _shooter.configure(Constants.CubeShooter.SHOOTER_CONFIG);
        _shooter.configureClosedLoop(Constants.CubeShooter.CONTROLLER_CONFIG_SHOOTER);
    }

    public void setWristSpeed(double speed) {
        _wrist.setPercentOutput(speed);
    }

    public void setShooterSpeed(double speed) {
        _shooter.setPercentOutput(speed);
    }

    public void setWristAngle(double angle) {
        metric("Set Angle", angle);
        metric("Set rotations", Units.radiansToRotations(angle * Constants.CubeShooter.GEAR_RATIO));
        _wrist.setMotionMagic(Units.radiansToRotations(angle * Constants.CubeShooter.GEAR_RATIO));
    }

    public void setWristEncoderRotation(double rotation) {
        _wrist.setRotorPosition(rotation);
    }

    public void setShooterRPS(double rps) {
        _shooter.setVelocity(rps);
    }

    public double getShooterRPS() {
        return _shooter.getVelocity().getValue();
    }

    public void zeroWrist() {
        setWristEncoderRotation(getBoreEncoderRotation());
    }

    public double getWristCurrent() {
        return _wrist.getStatorCurrent().getValue();
    }

    public double getWristEncoderRotation() {
        return _wrist.getPosition().getValue();
    }

    public double getBoreEncoderRotation() {
        return _wristEncoder.getDistance() + Constants.CubeShooter.ANKLE_OFFSET;
    }

    public double getWristAngleRadians() {
        return Units.rotationsToRadians(getWristEncoderRotation() / Constants.CubeShooter.GEAR_RATIO);
    }

    public boolean isCubeDetected() {
        return _proximitySensor.get();
    }

    public Pair<Double, Double> getShootingParameters(double distance, Nodes.Level level) {
        if (level == null) {
            return new Pair<>(0.3, Constants.CubeShooter.IDLE_ANGLE);
        }
        switch (level) {
            case HIGH:
                return new Pair<>(1.0, 0.21);
            case MIDDLE:
                if (distance < 0.5) {
                    return new Pair<>(0.5, 0.21);
                } else {
                    return new Pair<>(1.0, Constants.CubeShooter.IDLE_ANGLE);
                }
            case LOW:
                if (distance < 0.5) {
                    error("using low goal close");
                    return new Pair<>(0.2, Constants.CubeShooter.IDLE_ANGLE);
                } else {
                    error("using low goal far");
                    return new Pair<>(1.0, Constants.CubeShooter.IDLE_ANGLE);
                }
        }
        return new Pair<>(0.3, Constants.CubeShooter.IDLE_ANGLE);
    }

    public void updateDashboard() {
        metric("Wrist angle radians", getWristAngleRadians());
        metric("Shooter RPS", getShooterRPS());
        metric("Wrist Encoder rotations", getWristEncoderRotation());
        metric("Bore Encoder rotations", getBoreEncoderRotation());
        metric("Cube in Shooter", isCubeDetected());
    }
}
