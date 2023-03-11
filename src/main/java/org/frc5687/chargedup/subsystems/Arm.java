package org.frc5687.chargedup.subsystems;

import static org.frc5687.chargedup.Constants.Arm.*;

import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.RobotMap;
import org.frc5687.chargedup.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;

public class Arm extends OutliersSubsystem {
    private final OutliersTalon _talon;
    // private final HallEffect _upperHall;
    //    private final HallEffect _lowerHall;
    private final DutyCycleEncoder _absAngleEncoder;
    private final LinearSystemLoop<N2, N1, N1> _controlLoop;
    private final TrapezoidProfile.Constraints _contraints;
    private TrapezoidProfile.State _lastArmState;

    private Matrix<N1, N1> _u;

    public Arm(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.ARM, Constants.Arm.CAN_BUS, "arm");
        _talon.configure(Constants.Arm.CONFIG);

        // _upperHall = new HallEffect(RobotMap.DIO.TOP_HALL_ARM);
        //        _lowerHall = new HallEffect(RobotMap.DIO.BOTTOM_HALL_ARM);

        _absAngleEncoder = new DutyCycleEncoder(RobotMap.DIO.ARM_ENCODER);
        _absAngleEncoder.setDistancePerRotation(Math.PI); // 2:1 from output to encoder

        LinearSystem<N2, N1, N1> plant =
                LinearSystemId.createSingleJointedArmSystem(
                        DCMotor.getFalcon500(1),
                        INERTIA_ARM, // kg * m^2
                        GEAR_RATIO);
        KalmanFilter<N2, N1, N1> observer =
                new KalmanFilter<>(
                        Nat.N2(),
                        Nat.N1(),
                        plant,
                        VecBuilder.fill(MODEL_POSITION_NOISE, MODEL_VELOCITY_NOISE),
                        VecBuilder.fill(SENSOR_POSITION_NOISE),
                        kDt);
        LinearQuadraticRegulator<N2, N1, N1> controller =
                new LinearQuadraticRegulator<>(
                        plant, VecBuilder.fill(Q_POSITION, Q_VELOCITY), VecBuilder.fill(CONTROL_EFFORT), kDt);
        _controlLoop = new LinearSystemLoop<>(plant, controller, observer, CONTROL_EFFORT, kDt);

        _contraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        // we are setting velocity to 0 in the case that the arm was moving when starting up.
        _lastArmState = new TrapezoidProfile.State(getArmAngleRadians(), 0);
        _controlLoop.reset(VecBuilder.fill(getArmAngleRadians(), getArmVelocityRadPerSec()));
        _u = VecBuilder.fill(0);
    }

    public void calculateNextU() {
        Matrix<N2, N1> error =
                VecBuilder.fill(
                        _controlLoop.getNextR(0) - getArmAngleRadians(),
                        _controlLoop.getNextR(1) - getArmVelocityRadPerSec());
        _u = _controlLoop.clampInput(_controlLoop.getController().getK().times(error));
    }

    public void periodic() {
        super.periodic();
        calculateNextU();
        //        _controlLoop.correct(VecBuilder.fill(getArmAngleRadians()));
        //        _controlLoop.predict(kDt);
    }

    public void setArmSpeed(double speed) {
        _talon.setPercentOutput(speed);
    }

    public void setArmVoltage(double voltage) {
        // this normalized the 12 volts output between [-1, 1] ie. 6 volts / 12 volts = 50% speed.
        _talon.setVoltage(voltage);
    }

    public boolean getUpperHall() {
        return false;
    }

    public boolean getLowerHall() {
        //        return _lowerHall.get();
        return false;
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
        //        return OutliersTalon.rotationsToRadians(getEncoderRotation(),
        // Constants.Arm.GEAR_RATIO);
        return getAbsoluteArmEncoderAngle();
    }

    public double getPredictedArmAngleRadians() {
        return _controlLoop.getXHat(0);
    }

    public double getArmVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                OutliersTalon.rotationsPerSecToRPM(getEncoderRotationsPerSec(), GEAR_RATIO));
    }

    public double getPredictedArmVelocityRadPerSec() {
        return _controlLoop.getXHat(1);
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return _contraints;
    }

    public void setNextReference(TrapezoidProfile.State state) {
        _controlLoop.setNextR(state.position, state.velocity);
        _lastArmState = state;
    }

    public void setNextReference(double radians, double radPerSec) {
        _controlLoop.setNextR(radians, radPerSec);
    }

    public TrapezoidProfile.State getLastState() {
        return _lastArmState;
    }

    public double armFeedForward() {
        return ((ARM_LENGTH / 2.0) * (MOTOR_R * ARM_WEIGHT * 9.81) / (GEAR_RATIO * MOTOR_kT))
                * Math.cos(getArmAngleRadians() + (Math.PI / 2.0) - VERTICAL_ARM_ANGLE);
    }
    /**
     * Gets the next voltage to send to the falcon500.
     *
     * @return voltage
     */
    public double getNextVoltage() {
        //        return _controlLoop.getU(0) + armFeedForward();
        return _u.get(0, 0) + armFeedForward();
    }

    public void updateDashboard() {
        metric("Angle", getArmAngleRadians());
        metric("Next Voltage", getNextVoltage());
        metric("Estimated Angle", getPredictedArmAngleRadians());
        metric("Reference", _controlLoop.getNextR().get(0, 0));
        metric("Upper Hall triggered", getUpperHall());
        metric("Lower Hall triggered", getLowerHall());
    }
}
