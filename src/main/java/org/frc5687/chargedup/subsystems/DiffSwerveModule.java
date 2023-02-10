/* Team 5687 (C)5687-2022 */
package org.frc5687.chargedup.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.frc5687.chargedup.Constants;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.math.LinearSystems;
import org.frc5687.chargedup.util.Helpers;

import static org.frc5687.chargedup.Constants.DifferentialSwerveModule.*;

/**
 * Created 10/11/2020 by Dennis Slobodzian.
 *
 * <p>Wrapper class for a differential swerve module using LQR as the controller for azimuth angle
 * and wheel velocity of the module.
 */
public class DiffSwerveModule {
    private final OutliersTalon _rightFalcon;
    private final OutliersTalon _leftFalcon;
    private final DutyCycleEncoder _boreEncoder;
    private final Translation2d _positionVector;
    private final LinearSystemLoop<N3, N2, N3> _moduleControlLoop;
    private Matrix<N3, N1> _reference; // same thing as a set point.
    private Matrix<N2, N1> _u;

    private final double _encoderOffset;
    private final boolean _encoderInverted;

    private ControlState _controlState;

    private final SystemIO _systemIO;

    private TrapezoidProfile.State _angleSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State _wheelVelocityReference = new TrapezoidProfile.State();
    private final TrapezoidProfile.Constraints _profiledSteerConstraints;
    private final TrapezoidProfile.Constraints _profiledWheelConstraints;

    public DiffSwerveModule(
            ModuleConfiguration config, int leftMotorID, int rightMotorID, int encoderPort) {
        // setup azimuth bore encoder.
        _boreEncoder = new DutyCycleEncoder(encoderPort);
        _boreEncoder.setDistancePerRotation(2.0 * Math.PI);

        _encoderOffset = config.encoderOffset;
        _encoderInverted = config.encoderInverted;

        _reference = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);
        _positionVector = config.position;
        // setup both falcon motors.
        _leftFalcon = new OutliersTalon(leftMotorID, config.canBus, (config.moduleName + " Left"));
        _rightFalcon =
                new OutliersTalon(rightMotorID, config.canBus, (config.moduleName + " Right"));

        _leftFalcon.configure(CONFIG);
        _rightFalcon.configure(CONFIG);

        // Creates a Linear System of our Differential Swerve Module.
        LinearSystem<N3, N2, N3> swerveModuleModel =
                LinearSystems.createDifferentialSwerveModule(
                        DCMotor.getFalcon500(2),
                        INERTIA_STEER,
                        INERTIA_WHEEL,
                        GEAR_RATIO_STEER,
                        GEAR_RATIO_WHEEL);

        // Creates a Kalman Filter as our Observer for our module. Works since system is linear.
        KalmanFilter<N3, N2, N3> moduleObserver =
                new KalmanFilter<>(
                        Nat.N3(),
                        Nat.N3(),
                        swerveModuleModel,
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        MODEL_AZIMUTH_ANGLE_NOISE,
                                        MODEL_AZIMUTH_ANG_VELOCITY_NOISE,
                                        MODEL_WHEEL_ANG_VELOCITY_NOISE),
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        SENSOR_AZIMUTH_ANGLE_NOISE,
                                        SENSOR_AZIMUTH_ANG_VELOCITY_NOISE,
                                        SENSOR_WHEEL_ANG_VELOCITY_NOISE),
                        kDt);
        // Creates an LQR controller for our Swerve Module.
        LinearQuadraticRegulator<N3, N2, N3> moduleController =
                new LinearQuadraticRegulator<>(
                        swerveModuleModel,
                        // Q Vector/Matrix Maximum error tolerance
                        VecBuilder.fill(Q_AZIMUTH, Q_AZIMUTH_ANG_VELOCITY, Q_WHEEL_ANG_VELOCITY),
                        // R Vector/Matrix Maximum control effort.
                        VecBuilder.fill(CONTROL_EFFORT, CONTROL_EFFORT),
                        kDt);

        // Creates a LinearSystemLoop that contains the Model, Controller, Observer, Max Volts,
        // Update Rate.
        _moduleControlLoop =
                new LinearSystemLoop<>(
                        swerveModuleModel, moduleController, moduleObserver, VOLTAGE, kDt);

        //         Initializes the vectors and matrices.
        _moduleControlLoop.reset(VecBuilder.fill(0, 0, 0));
        _u = VecBuilder.fill(0, 0);
        // boolean for if we want the modules to be running as we set voltage in the periodic loop.
        _systemIO = new SystemIO();
        _controlState = ControlState.OFF;

        _profiledSteerConstraints = new TrapezoidProfile.Constraints(
                Constants.DifferentialSwerveModule.MAX_ANGULAR_VELOCITY,
                Constants.DifferentialSwerveModule.MAX_ANGULAR_ACCELERATION);
        _profiledWheelConstraints = new TrapezoidProfile.Constraints(
                Constants.DifferentialSwerveModule.MAX_MODULE_ACCELERATION,
                Constants.DifferentialSwerveModule.MAX_MODULE_JERK);
    }
//
    public ControlState getControlState() {
        return _controlState;
    }

    public void setControlState(ControlState state) {
        _controlState = state;
    }

    public synchronized void readInputs() {
        _systemIO.leftVelocityTicksPer100ms = _leftFalcon.getSelectedSensorVelocity(0);
        _systemIO.leftPositionTicks = _leftFalcon.getSelectedSensorPosition(0);
        _systemIO.rightVelocityTicksPer100ms = _rightFalcon.getSelectedSensorVelocity(0);
        _systemIO.rightPositionTicks = _rightFalcon.getSelectedSensorPosition(0);

        _systemIO.moduleAngle = getEncoderAngle();
        _systemIO.moduleAzimuthAngularVelocity = getAzimuthAngularVelocity();
        _systemIO.moduleWheelAngularVelocity = getWheelAngularVelocity();
    }
    // periodic loop runs at 5ms.
    public void periodic() {
        // read inputs from falcon;
        readInputs();
        switch (_controlState) {
            case OFF:
                stop();
                break;
            case STATE_CONTROL:
                // sets the next reference / setpoint.
                _moduleControlLoop.setNextR(_reference);
                // updates the kalman filter with new data points.
                _moduleControlLoop.correct(
                        VecBuilder.fill(
                                getModuleAngle(),
                                getAzimuthAngularVelocity(),
                                getWheelAngularVelocity()));
                predict();

                setLeftFalconVoltage(getLeftNextVoltage());
                setRightFalconVoltage(getRightNextVoltage());
                break;
        }
    }
    /**
     * Calculated the profiled reference with angle wrapping.
     * @return Vector (r-x) vector with profiled values.
     */
    private Matrix<N3, N1> profiledReference(Matrix<N3, N1> reference, Matrix<N3, N1> xHat) {
        double errorBound = (Math.PI - (-Math.PI)) / 2.0;
        double angleMinimumGoalDistance = MathUtil.inputModulus(
                reference.get(0,0) - getModuleAngle(),
                -errorBound,
                errorBound
        );
        double angleMinimumSetpointDistance = MathUtil.inputModulus(
                _angleSetpoint.position - getModuleAngle(),
                -errorBound,
                errorBound
        );
        reference.set(0, 0, angleMinimumGoalDistance + getModuleAngle());
        _angleSetpoint.position = angleMinimumSetpointDistance + getModuleAngle();

        var steerProfile = new TrapezoidProfile(
                _profiledSteerConstraints,
                new TrapezoidProfile.State(
                        reference.get(0,0),
                        reference.get(1, 0)),
                _angleSetpoint);
        _angleSetpoint = steerProfile.calculate(Constants.DifferentialSwerveModule.kDt);
//        var wheelProfile = new TrapezoidProfile(
//                _profiledWheelConstraints,
//                new TrapezoidProfile.State(_reference.get(2,0),0),
//                _wheelVelocityReference
//        );
//        _wheelVelocityReference = wheelProfile.calculate(Constants.DifferentialSwerveModule.kDt);

        Matrix<N3, N1> error = reference.minus(xHat);
        return VecBuilder.fill(
                _angleSetpoint.position - getModuleAngle(),
                _angleSetpoint.velocity - getAzimuthAngularVelocity(),
                error.get(2,0));
//                _wheelVelocityReference.position - getWheelAngularVelocity());
    }
    /**
     * wraps angle so that absolute encoder can be continuous. (i.e) No issues when switching
     * between -PI and PI as they are the same point but different values.
     *
     * @param reference is the Matrix that contains the reference wanted such as [Math.PI, 0, 100].
     * @param xHat is the predicted states of our system. [Azimuth Angle, Azimuth Angular Velocity,
     *     Wheel Angular Velocity].
     */
    private Matrix<N3, N1> wrapAngle(Matrix<N3, N1> reference, Matrix<N3, N1> xHat) {
        double angleError = reference.get(0, 0) - getModuleAngle();
        double positionError = MathUtil.inputModulus(angleError, -Math.PI, Math.PI);
        Matrix<N3, N1> error = reference.minus(xHat);
        return VecBuilder.fill(positionError, error.get(1, 0), error.get(2, 0));
    }
    // use custom predict() function for as absolute encoder azimuth angle and the angular velocity
    // of the module need to be continuous.
    private void predict() {
        // creates our input of voltage to our motors of u = K(r-x) but need to wrap angle to be
        // continuous
        _u =
                _moduleControlLoop.clampInput(
                        _moduleControlLoop
                                .getController()
                                .getK()
                                .times(profiledReference(
                                        _moduleControlLoop.getNextR(),
                                        _moduleControlLoop.getXHat()
                                        ))
//                                        wrapAngle(
//                                                _moduleControlLoop.getNextR(),
//                                                _moduleControlLoop.getXHat()))
                                .plus(
                                        _moduleControlLoop
                                                .getFeedforward()
                                                .calculate(_moduleControlLoop.getNextR())));
        _moduleControlLoop.getObserver().predict(_u, kDt);
    }

    public void start() {
        setControlState(ControlState.STATE_CONTROL);
    }

    public void stop() {
        setLeftFalconVoltage(0.0);
        setRightFalconVoltage(0.0);
    }

    // location is x, y position w.r.t robot frame
    public Translation2d getModuleLocation() { return _positionVector; }
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getWheelDistance(), new Rotation2d(getModuleAngle()));
    }

    public void setRightFalconVoltage(double voltage) {
        double limVoltage = Helpers.limit(voltage, -VOLTAGE, VOLTAGE);
        _rightFalcon.set(TalonFXControlMode.PercentOutput, limVoltage / VOLTAGE);
    }

    public void setLeftFalconVoltage(double voltage) {
        double limVoltage = Helpers.limit(voltage, -VOLTAGE, VOLTAGE);
        _leftFalcon.set(TalonFXControlMode.PercentOutput, limVoltage / VOLTAGE);
    }

    public double getEncoderAngle() {
        return Helpers.boundHalfAngle(
                ((_encoderInverted ? (-1.0) : 1.0) * _boreEncoder.getDistance() % (2.0 * Math.PI))
                        - _encoderOffset,
                true);
    }

    public double getModuleAngle() {
        return _systemIO.moduleAngle;
    }

    public double getWheelAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                        getLeftFalconRPM() / GEAR_RATIO_WHEEL
                                - getRightFalconRPM() / GEAR_RATIO_WHEEL)
                / 2.0;
    }

    public double getWheelVelocity() {
        return getWheelAngularVelocity() * WHEEL_RADIUS; // Meters per sec.
    }

    public double getWheelDistance() {
        return ((getLeftFalconDistanceRadians() / GEAR_RATIO_WHEEL -
                getRightFalconDistanceRadians() / GEAR_RATIO_WHEEL) / 2.0) * WHEEL_RADIUS;
    }
    public double getPredictedWheelVelocity() {
        return getPredictedWheelAngularVelocity() * WHEEL_RADIUS;
    }

    public double getAzimuthAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                        getLeftFalconRPM() / GEAR_RATIO_STEER
                                + getRightFalconRPM() / GEAR_RATIO_STEER)
                / 2.0;
    }

    public double getRightFalconRPM() {
        return OutliersTalon.ticksPer100msToRPM(_systemIO.rightVelocityTicksPer100ms, 1.0);
    }
    public double getRightFalconDistanceRadians() {
        return OutliersTalon.ticksToRadians(_systemIO.rightPositionTicks, 1.0);
    }
    public double getLeftFalconRPM() {
        return OutliersTalon.ticksPer100msToRPM(_systemIO.leftVelocityTicksPer100ms, 1.0);
    }
    public double getLeftFalconDistanceRadians() {
        return OutliersTalon.ticksToRadians(_systemIO.leftPositionTicks, 1.0);
    }

    public void resetEncoders() {
        _leftFalcon.setSelectedSensorPosition(0);
        _rightFalcon.setSelectedSensorPosition(0);
    }

    public double getLeftVoltage() {
        return _leftFalcon.getMotorOutputVoltage();
    }

    public double getRightVoltage() {
        return _rightFalcon.getMotorOutputVoltage();
    }

    public double getPredictedAzimuthAngularVelocity() {
        return _moduleControlLoop.getObserver().getXhat(1);
    }

    public double getPredictedWheelAngularVelocity() {
        return _moduleControlLoop.getXHat(2);
    }

    public double getPredictedAzimuthAngle() {
        return _moduleControlLoop.getXHat(0);
    }

    public double getReferenceWheelAngularVelocity() {
        return _moduleControlLoop.getNextR(2);
    }

    public void setReference(Matrix<N3, N1> reference) {
        _reference = reference;
    }

    /**
     * gets the wanted voltage from our control law. u = K(r-x) our control law is slightly
     * different as we need to be continuous. Check method predict() for calculations.
     *
     * @return left wanted voltage
     */
    public double getLeftNextVoltage() {
        return _u.get(0, 0);
    }

    public double getRightNextVoltage() {
        return _u.get(1, 0);
    }

    public double getLeftCurrent() {
        return _leftFalcon.getSupplyCurrent();
    }

    public double getRightCurrent() {
        return _rightFalcon.getSupplyCurrent();
    }

    public double getReferenceModuleAngle() {
        return _moduleControlLoop.getNextR(0);
    }

    public double getReferenceModuleAngularVelocity() {
        return _moduleControlLoop.getNextR(1);
    }

    public double getReferenceWheelVelocity() {
        return _moduleControlLoop.getNextR(2);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocity(), new Rotation2d(getModuleAngle()));
    }

    /**
     * Sets the state of the module and sends the voltages wanted to the motors.
     *
     * @param state is the desired swerve module state.
     */
    public void setModuleState(SwerveModuleState state) {
        if (state.speedMetersPerSecond != 0) {
            setReference(
                    VecBuilder.fill(
                            state.angle.getRadians(),
                            0,
                            state.speedMetersPerSecond / WHEEL_RADIUS));
        } else {
            setReference(
                    VecBuilder.fill(
                            getModuleAngle(), 0, state.speedMetersPerSecond / WHEEL_RADIUS));
        }
    }

    /**
     * sets the modules to take the shorted path to the newest state.
     *
     * @param state azimuth angle in radians and velocity of wheel in meters per sec.
     */
    public void setIdealState(SwerveModuleState state) {
        var delta = state.angle.minus(new Rotation2d(getModuleAngle()));
        if (Math.abs(delta.getDegrees()) > 95.0) {
            setModuleState(new SwerveModuleState(
            -state.speedMetersPerSecond,
            state.angle.rotateBy(Rotation2d.fromDegrees(180.0))));
        } else {
            setModuleState(new SwerveModuleState(state.speedMetersPerSecond, state.angle));
        }
    }

    public enum ControlState {
        OFF(0),
        STATE_CONTROL(1);
        private final int _value;

        ControlState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    private static class SystemIO {
        // Falcon 500 sensor inputs
        public double leftVelocityTicksPer100ms;
        public double leftPositionTicks;
        public double rightVelocityTicksPer100ms;
        public double rightPositionTicks;
        // State Space Sensor Inputs
        public double moduleAngle;
        public double moduleAzimuthAngularVelocity;
        public double moduleWheelAngularVelocity;
    }

    public static class ModuleConfiguration {
        public String moduleName = "";

        public Translation2d position = new Translation2d();

        public double encoderOffset = 0.0;
        public boolean encoderInverted = true;

        public String canBus = "rio";
    }
}
