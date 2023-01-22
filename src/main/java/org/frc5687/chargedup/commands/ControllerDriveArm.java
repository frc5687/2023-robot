package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.Constants;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ControllerDriveArm extends OutliersCommand{

    private Arm _arm;
    private OI _oi;

    public ControllerDriveArm(Arm arm, OI oi) {
        _arm = arm;
        _oi = oi;
    }

    private final TrapezoidProfile.Constraints _constraints =
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(45),
            Units.degreesToRadians(90)); // Max arm speed and acceleration.
    TrapezoidProfile.State _lastProfiledReference = new TrapezoidProfile.State();

    private final LinearSystem<N2, N1, N1> _armPlant =
        LinearSystemId.createSingleJointedArmSystem(_arm., _arm.getEncoderTicks(), _arm.getArmAngleRadians());

    private final KalmanFilter<N2, N1, N1> _observer =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            _armPlant,
            VecBuilder.fill(0.015, 0.17),
            VecBuilder.fill(0.01),0.020);

    private final LinearQuadraticRegulator<N2, N1, N1> _controller =
        new LinearQuadraticRegulator<>(
            _armPlant,
            VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)),
            VecBuilder.fill(12.0), 0.020);

    private final LinearSystemLoop<N2, N1, N1> _loop =
            new LinearSystemLoop<>(_armPlant, _controller, _observer, 12.0, 0.020);
    
    // An encoder set up to measure arm position in radians.
    private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

    private final MotorController m_motor = new PWMSparkMax(kMotorPort);

    @Override
    public void initialize() {
    _loop.reset(VecBuilder.fill(_encoder.getDistance(), _encoder.getRate()));

    _lastProfiledReference =
        new TrapezoidProfile.State(_encoder.getDistance(), _encoder.getRate());
    }

    @Override
    public void execute() {
        // Sets the target position of our arm. This is similar to setting the setpoint of a
        // PID controller.
        TrapezoidProfile.State goal;
        if (_oi.raiseArm()) {
            // the trigger is pressed, so we go to the high goal.
            goal = new TrapezoidProfile.State(Constants.Arm.raisedPosition, 0.0);
        } else {
            // Otherwise, we go to the low goal
            goal = new TrapezoidProfile.State(Constants.Arm.loweredPosition, 0.0);
        }
        // Step our TrapezoidalProfile forward 20ms and set it as our next reference
        _lastProfiledReference =
            (new TrapezoidProfile(_constraints, goal, _lastProfiledReference)).calculate(0.020);
        _loop.setNextR(_lastProfiledReference.position, _lastProfiledReference.velocity);

        // Correct our Kalman filter's state vector estimate with encoder data.
        _loop.correct(VecBuilder.fill(m_encoder.getDistance()));

        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state with out Kalman filter.
        _loop.predict(0.020);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = _loop.getU(0);
        _motor.setVoltage(nextVoltage);
    }
}
