package org.frc5687.chargedup.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.ejml.dense.row.mult.VectorVectorMult_CDRM;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;

import com.ctre.phoenix.motorcontrol.ControlMode;

import static org.frc5687.chargedup.Constants.Arm.*;

public class Arm extends OutliersSubsystem{
    private final OutliersTalon _talon;
    private final HallEffect _upperHall;
    private final HallEffect _lowerHall;
    private final LinearSystemLoop<N2, N1, N1> _controlLoop;

    private final TrapezoidProfile.Constraints _contraints;

    public Arm(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.ARM, Constants.Arm.CAN_BUS, "arm");
        _talon.configure(Constants.Arm.CONFIG);
        _upperHall = new HallEffect(RobotMap.DIO.TOP_HALL_ARM);
        _lowerHall = new HallEffect(RobotMap.DIO.BOTTOM_HALL_ARM);
        LinearSystem<N2, N1, N1> plant = LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getFalcon500(1),
                INERTIA_ARM, // kg * m^2
                GEAR_RATIO
                );
        KalmanFilter<N2, N1, N1> observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                plant,
                VecBuilder.fill(MODEL_POSITION_NOISE, MODEL_VELOCITY_NOISE),
                VecBuilder.fill(SENSOR_POSITION_NOISE),
                kDt
        );
        LinearQuadraticRegulator<N2, N1, N1> controller = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(Q_POSITION, Q_VELOCITY),
                VecBuilder.fill(CONTROL_EFFORT),
                kDt
        );

        _controlLoop = new LinearSystemLoop<>(
                plant,
                controller,
                observer,
                CONTROL_EFFORT,
                kDt
        );

        _contraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        _controlLoop.reset(VecBuilder.fill(getArmAngleRadians(), getArmVelocityRadPerSec()));
    }

    @Override
    public void periodic() {
        super.periodic();
        // update our kalman filter.
//        if (_upperHall.get() && _controlLoop.getU(0) > 0) {
//            _controlLoop.reset(VecBuilder.fill(getArmAngleRadians(), 0));
//            _controlLoop.setNextR(VecBuilder.fill(getArmAngleRadians(), 0));
//            //reset encoder
//            _talon.setSelectedSensorPosition(OutliersTalon.radiansToTicks(UPPER_HALL_RAD, GEAR_RATIO));
//        } else if (_lowerHall.get() && _controlLoop.getU(0) > 0) {
//            _controlLoop.reset(VecBuilder.fill(getArmAngleRadians(), 0));
//            _controlLoop.setNextR(VecBuilder.fill(getArmAngleRadians(), 0));
//            //reset encoder
//            _talon.setSelectedSensorPosition(OutliersTalon.radiansToTicks(LOWER_HALL_RAD, GEAR_RATIO));
//
//        }
        _controlLoop.correct(VecBuilder.fill(getArmAngleRadians()));
        _controlLoop.predict(kDt);
    }

    public void setArmSpeed(double speed) {
        _talon.set(ControlMode.PercentOutput, speed);
    }

    public void setArmVoltage(double voltage) {
        // this normalized the 12 volts output between [-1, 1] ie. 6 volts / 12 volts = 50% speed.
        setArmSpeed(voltage / CONTROL_EFFORT);
    }

    public boolean getUpperHall() {
        return _upperHall.get();
    }

    public boolean getLowerHall() {
        return _lowerHall.get();
    }

    public double getEncoderTicks() {
        return _talon.getSelectedSensorPosition();
    }

    public double getEncoderTicksPer100ms() {
        return _talon.getSelectedSensorVelocity();
    }

    public double getArmAngleRadians() {
        return OutliersTalon.ticksToRadians(getEncoderTicks(), Constants.Arm.GEAR_RATIO);
    }
    public double getPredictedArmAngleRadians() {
        return _controlLoop.getXHat(0);
    }
    public double getArmVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                OutliersTalon.ticksPer100msToRPM(getEncoderTicksPer100ms(), GEAR_RATIO)
        );
    }
    public double getPredictedArmVelocityRadPerSec() {
        return _controlLoop.getXHat(1);
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return _contraints;
    }

    public void setNextReference(TrapezoidProfile.State state) {
        _controlLoop.setNextR(state.position, state.velocity);
    }

    public void setNextReference(double radians, double radPerSec) {
        _controlLoop.setNextR(radians, radPerSec);
    }

    /**
     * Gets the next voltage to send to the falcon500.
     * @return voltage
     */
    public double getNextVoltage() {
        return _controlLoop.getU(0);
    }

    public void updateDashboard() {
        metric("Angle", getArmAngleRadians());
        metric("Next Voltage", getNextVoltage());
        metric("Estimated Angle", getPredictedArmAngleRadians());
        metric("Reference", _controlLoop.getNextR().toString());
        metric("Upper Hall triggered", getUpperHall());
        metric("Lower Hall triggered", getUpperHall());
    }
}
