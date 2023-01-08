/* Team 5687 (C)5687-2022 */
package org.frc5687.chargedup.subsystems;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.util.Helpers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


/**
 * Created 10/11/2020 by Dennis Slobodzian.
 *
 * <p>Wrapper class for a differential swerve module using LQR as the controller for azimuth angle
 * and wheel velocity of the module.
 */
public class DiffSwerveModule {
    private final TalonFX _rightFalcon;
    private final TalonFX _leftFalcon;
    private final DutyCycleEncoder _boreEncoder;
    private final Translation2d _positionVector;
    private final LinearSystemLoop<N3, N2, N3> _swerveControlLoop;
    private Matrix<N3, N1> _reference; // same thing as a set point.
    private Matrix<N2, N1> _u;

//    private TrapezoidProfile.State _angleReference = new TrapezoidProfile.State();
//    private TrapezoidProfile.State _wheelVelocityReference = new TrapezoidProfile.State();
//    private final TrapezoidProfile.Constraints _profiledSteerConstraints;
//    private final TrapezoidProfile.Constraints _profiledWheelConstraints;

    private final double _encoderOffset;
    private boolean _running;
    private boolean _encoderInverted;

    public DiffSwerveModule(
            Translation2d positionVector,
            int leftMotorID,
            int rightMotorID,
            int encoderNum,
            double encoderOffset,
            boolean encoderInverted) {
        // setup azimuth bore encoder.
        _boreEncoder = new DutyCycleEncoder(encoderNum);
        _boreEncoder.setDistancePerRotation(2.0 * Math.PI);
        _encoderOffset = encoderOffset;
        _encoderInverted = encoderInverted;

        _reference = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);
        _positionVector = positionVector;

        // setup both falcon motors.
        _leftFalcon = new TalonFX(leftMotorID, "DriveTrain");
        _rightFalcon = new TalonFX(rightMotorID, "DriveTrain");
        _leftFalcon.configFactoryDefault();
        _rightFalcon.configFactoryDefault();
        _rightFalcon.setInverted(false);
        _leftFalcon.setInverted(false);
        _rightFalcon.setSensorPhase(false);
        _leftFalcon.setSensorPhase(false);
        // Model assumes the motors are in brake mode so do not change.
        _rightFalcon.setNeutralMode(NeutralMode.Brake);
        _leftFalcon.setNeutralMode(NeutralMode.Brake);

        _rightFalcon.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor, 0, Constants.DifferentialSwerveModule.TIMEOUT);
        _leftFalcon.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor, 0, Constants.DifferentialSwerveModule.TIMEOUT);
        _rightFalcon.configForwardSoftLimitEnable(false);
        _leftFalcon.configForwardSoftLimitEnable(false);

        _rightFalcon.configVoltageCompSaturation(
                Constants.DifferentialSwerveModule.VOLTAGE,
                Constants.DifferentialSwerveModule.TIMEOUT);
        _leftFalcon.configVoltageCompSaturation(
                Constants.DifferentialSwerveModule.VOLTAGE,
                Constants.DifferentialSwerveModule.TIMEOUT);

        _rightFalcon.enableVoltageCompensation(true);
        _leftFalcon.enableVoltageCompensation(true);

        _rightFalcon.setStatusFramePeriod(
                StatusFrame.Status_1_General, 5, Constants.DifferentialSwerveModule.TIMEOUT);
        _leftFalcon.setStatusFramePeriod(
                StatusFrame.Status_1_General, 5, Constants.DifferentialSwerveModule.TIMEOUT);

        _rightFalcon.setStatusFramePeriod(
                StatusFrame.Status_2_Feedback0, 5, Constants.DifferentialSwerveModule.TIMEOUT);
        _leftFalcon.setStatusFramePeriod(
                StatusFrame.Status_2_Feedback0, 5, Constants.DifferentialSwerveModule.TIMEOUT);

        _rightFalcon.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms, Constants.DifferentialSwerveModule.TIMEOUT);
        _leftFalcon.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms, Constants.DifferentialSwerveModule.TIMEOUT);

//        _rightFalcon.configVelocityMeasurementWindow(64, Constants.DifferentialSwerveModule.TIMEOUT);
//        _leftFalcon.configVelocityMeasurementWindow(64, Constants.DifferentialSwerveModule.TIMEOUT);

        _rightFalcon.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        Constants.DifferentialSwerveModule.ENABLE_CURRENT_LIMIT,
                        Constants.DifferentialSwerveModule.CURRENT_LIMIT,
                        Constants.DifferentialSwerveModule.CURRENT_THRESHOLD,
                        Constants.DifferentialSwerveModule.CURRENT_TRIGGER_TIME));
        _leftFalcon.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        Constants.DifferentialSwerveModule.ENABLE_CURRENT_LIMIT,
                        Constants.DifferentialSwerveModule.CURRENT_LIMIT,
                        Constants.DifferentialSwerveModule.CURRENT_THRESHOLD,
                        Constants.DifferentialSwerveModule.CURRENT_TRIGGER_TIME));

        // Creates a Linear System of our Differential Swerve Module.
        LinearSystem<N3, N2, N3> swerveModuleModel =
                createDifferentialSwerveModule(
                        DCMotor.getFalcon500(2),
                        Constants.DifferentialSwerveModule.INERTIA_STEER,
                        Constants.DifferentialSwerveModule.INERTIA_WHEEL,
                        Constants.DifferentialSwerveModule.GEAR_RATIO_STEER,
                        Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL);

        // Creates a Kalman Filter as our Observer for our module. Works since system is linear.
        KalmanFilter<N3, N2, N3> swerveObserver =
                new KalmanFilter<>(
                        Nat.N3(),
                        Nat.N3(),
                        swerveModuleModel,
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        Constants.DifferentialSwerveModule
                                                .MODEL_AZIMUTH_ANGLE_NOISE,
                                        Constants.DifferentialSwerveModule
                                                .MODEL_AZIMUTH_ANG_VELOCITY_NOISE,
                                        Constants.DifferentialSwerveModule
                                                .MODEL_WHEEL_ANG_VELOCITY_NOISE),
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        Constants.DifferentialSwerveModule
                                                .SENSOR_AZIMUTH_ANGLE_NOISE,
                                        Constants.DifferentialSwerveModule
                                                .SENSOR_AZIMUTH_ANG_VELOCITY_NOISE,
                                        Constants.DifferentialSwerveModule
                                                .SENSOR_WHEEL_ANG_VELOCITY_NOISE),
                        Constants.DifferentialSwerveModule.kDt);
        // Creates an LQR controller for our Swerve Module.
        LinearQuadraticRegulator<N3, N2, N3> swerveController =
                new LinearQuadraticRegulator<>(
                        swerveModuleModel,
                        // Q Vector/Matrix Maximum error tolerance
                        VecBuilder.fill(
                                Constants.DifferentialSwerveModule.Q_AZIMUTH,
                                Constants.DifferentialSwerveModule.Q_AZIMUTH_ANG_VELOCITY,
                                Constants.DifferentialSwerveModule.Q_WHEEL_ANG_VELOCITY),
                        // R Vector/Matrix Maximum control effort.
                        VecBuilder.fill(
                                Constants.DifferentialSwerveModule.CONTROL_EFFORT,
                                Constants.DifferentialSwerveModule.CONTROL_EFFORT),
                        Constants.DifferentialSwerveModule.kDt);

        // Creates a LinearSystemLoop that contains the Model, Controller, Observer, Max Volts,
        // Update Rate.
        _swerveControlLoop =
                new LinearSystemLoop<>(
                        swerveModuleModel,
                        swerveController,
                        swerveObserver,
                        12.0,
                        Constants.DifferentialSwerveModule.kDt);

        // Initializes the vectors and matrices.
        _swerveControlLoop.reset(VecBuilder.fill(0, 0, 0));
        _u = VecBuilder.fill(0, 0);

        // boolean for if we want the modules to be running as we set voltage in the periodic loop.
        _running = false;
    }

    /**
     * wraps angle so that absolute encoder can be continues. (i.e) No issues when switching between
     * -PI and PI as they are the same point but different values.
     *
     * @param reference is the Matrix that contains the reference wanted such as [Math.PI, 0, 100].
     * @param xHat is the predicted states of our system. [Azimuth Angle, Azimuth Angular Velocity,
     *     Wheel Angular Velocity].
     * @param minAngle is the minimum angle in our case -PI.
     * @param maxAngle is the maximum angle in our case PI.
     */
    private Matrix<N3, N1> wrapAngle(
            Matrix<N3, N1> reference, Matrix<N3, N1> xHat, double minAngle, double maxAngle) {
        double angleError = reference.get(0, 0) - getModuleAngle();
        double positionError = MathUtil.inputModulus(angleError, minAngle, maxAngle);
        Matrix<N3, N1> error = reference.minus(xHat);
        return VecBuilder.fill(positionError, error.get(1, 0), error.get(2, 0));
    }

    // periodic loop runs at 5ms.
    public void periodic() {
        // sets the next reference / setpoint.
        _swerveControlLoop.setNextR(_reference);
        // updates the kalman filter with new data points.
        _swerveControlLoop.correct(
                VecBuilder.fill(
                        getModuleAngle(), getAzimuthAngularVelocity(), getWheelAngularVelocity()));
        // predict step of kalman filter.
        predict();
        if (_running) {
            setLeftFalconVoltage(getLeftNextVoltage());
            setRightFalconVoltage(getRightNextVoltage());
        }
    }

    // use custom predict() function for as absolute encoder azimuth angle and the angular velocity
    // of the module need to be continuous.
    private void predict() {
        // creates our input of voltage to our motors of u = K(r-x) but need to wrap angle to be
        // continuous
        _u =
                _swerveControlLoop.clampInput(
                        _swerveControlLoop
                                .getController()
                                .getK()
                                .times(//profiledReference())
                                        wrapAngle(
                                                _swerveControlLoop.getNextR(),
                                                _swerveControlLoop.getXHat(),
                                                -Math.PI,
                                                Math.PI))
                                .plus(
                                        VecBuilder.fill(
                                                Constants.DifferentialSwerveModule.FEED_FORWARD
                                                        * _reference.get(2, 0),
                                                -Constants.DifferentialSwerveModule.FEED_FORWARD
                                                        * _reference.get(2, 0))));
        _swerveControlLoop.getObserver().predict(_u, Constants.DifferentialSwerveModule.kDt);
    }

    public void start() {
        _running = true;
    }

    public void stop() {
        _running = false;
    }

    // this is the location of the module with respect to the robot.
    public Translation2d getModuleLocation() {
        return _positionVector;
    }

    // this is the location of the module with respect to the field.
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getWheelDistanceTraveled(), new Rotation2d(getModuleAngle()));
    }

    public void setRightFalconVoltage(double voltage) {
        double limVoltage =
                Helpers.limit(
                        voltage,
                        -Constants.DifferentialSwerveModule.VOLTAGE,
                        Constants.DifferentialSwerveModule.VOLTAGE);
        _rightFalcon.set(
                TalonFXControlMode.PercentOutput,
                limVoltage / Constants.DifferentialSwerveModule.VOLTAGE);
    }

    public void setLeftFalconVoltage(double voltage) {
        double limVoltage =
                Helpers.limit(
                        voltage,
                        -Constants.DifferentialSwerveModule.VOLTAGE,
                        Constants.DifferentialSwerveModule.VOLTAGE);
        _leftFalcon.set(
                TalonFXControlMode.PercentOutput,
                limVoltage / Constants.DifferentialSwerveModule.VOLTAGE);
    }

    public double getModuleAngle() {
        return Helpers.boundHalfAngle(
                ((_encoderInverted ? (-1.0) : 1.0)
                        * _boreEncoder.getDistance() % (2.0 * Math.PI)) - _encoderOffset,
                true);
    }

    public double getWheelAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                getLeftFalconRPM() / Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL
                        - getRightFalconRPM()
                        / Constants.DifferentialSwerveModule.GEAR_RATIO_WHEEL)
                / 2.0;
    }

    public double getWheelDistanceTraveled() {
        return (getLeftFalconDistanceMeters() - getRightFalconDistanceMeters()) / 2.0;
    }

    public double getWheelVelocity() {
        return getWheelAngularVelocity()
                * Constants.DifferentialSwerveModule.WHEEL_RADIUS; // Meters per sec.
    }

    public double getPredictedWheelVelocity() {
        return getPredictedWheelAngularVelocity() * Constants.DifferentialSwerveModule.WHEEL_RADIUS;
    }

    public double getAzimuthAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(
                getLeftFalconRPM() / Constants.DifferentialSwerveModule.GEAR_RATIO_STEER
                        + getRightFalconRPM()
                        / Constants.DifferentialSwerveModule.GEAR_RATIO_STEER)
                / 2.0;
    }

    public double getRightFalconRPM() {
        return _rightFalcon.getSelectedSensorVelocity()
                / Constants.DifferentialSwerveModule.TICKS_TO_ROTATIONS
                * Constants.DifferentialSwerveModule.FALCON_RATE;
    }

    public double getRightFalconDistanceMeters() {
        return (_rightFalcon.getSelectedSensorPosition() /
                Constants.DifferentialSwerveModule.TICKS_TO_ROTATIONS) *
                Constants.DifferentialSwerveModule.WHEEL_RADIUS;
    }

    public double getLeftFalconRPM() {
        return _leftFalcon.getSelectedSensorVelocity()
                / Constants.DifferentialSwerveModule.TICKS_TO_ROTATIONS
                * Constants.DifferentialSwerveModule.FALCON_RATE;
    }
    public double getLeftFalconDistanceMeters() {
        return (_leftFalcon.getSelectedSensorPosition() /
                Constants.DifferentialSwerveModule.TICKS_TO_ROTATIONS) *
                Constants.DifferentialSwerveModule.WHEEL_RADIUS;
    }

    public double getLeftVoltage() {
        return _leftFalcon.getMotorOutputVoltage();
    }

    public double getRightVoltage() {
        return _rightFalcon.getMotorOutputVoltage();
    }

    public double getPredictedAzimuthAngularVelocity() {
        return _swerveControlLoop.getObserver().getXhat(1);
    }

    public double getPredictedWheelAngularVelocity() {
        return _swerveControlLoop.getXHat(2);
    }

    public double getPredictedAzimuthAngle() {
        return _swerveControlLoop.getXHat(0);
    }

    public double getReferenceWheelAngularVelocity() {
        return _swerveControlLoop.getNextR(2);
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
        return _swerveControlLoop.getNextR(0);
    }

    public double getReferenceModuleAngularVelocity() {
        return _swerveControlLoop.getNextR(1);
    }

    public double getReferenceWheelVelocity() {
        return _swerveControlLoop.getNextR(2);
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
//        SmartDashboard.putNumber("new state angle", state.angle.getRadians());

        setReference(
                VecBuilder.fill(
                        state.angle.getRadians(),
                        0,
                        state.speedMetersPerSecond
                                / Constants.DifferentialSwerveModule.WHEEL_RADIUS));
    }

    /**
     * sets the modules to take the shorted path to the newest state.
     *
     * @param state azimuth angle in radians and velocity of wheel in meters per sec.
     */
    public void setIdealState(SwerveModuleState state) {
        setModuleState(SwerveModuleState.optimize(state, new Rotation2d(getModuleAngle())));
    }

    /**
     * Creates a StateSpace model of a differential swerve module.
     *
     * @param motor is the motor used.
     * @param Js is the Moment of Inertia of the steer component.
     * @param Jw is the Moment of Inertia of the wheel component.
     * @param Gs is the Gear Ratio of the steer.
     * @param Gw is the Gear Ratio of the wheel.
     * @return LinearSystem of state space model.
     */
    private static LinearSystem<N3, N2, N3> createDifferentialSwerveModule(
            DCMotor motor, double Js, double Jw, double Gs, double Gw) {
        var Cs = -((Gs * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Js));
        var Cw = -((Gw * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Jw));
        var Vs = 0.5 * ((Gs * motor.KtNMPerAmp) / (motor.rOhms * Js));
        var Vw = 0.5 * ((Gw * motor.KtNMPerAmp) / (motor.rOhms * Jw));

        var A =
                Matrix.mat(Nat.N3(), Nat.N3())
                        .fill(0.0, 1.0, 0.0, 0.0, Gs * Cs, 0.0, 0.0, 0.0, Gw * Cw);
        var B = Matrix.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, Vs, Vs, Vw, -Vw);
        var C = Matrix.mat(Nat.N3(), Nat.N3()).fill(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        var D =
                Matrix.mat(Nat.N3(), Nat.N2())
                        .fill(
                                0.0, 0.0,
                                0.0, 0.0,
                                0.0, 0.0);
        return new LinearSystem<>(A, B, C, D);
    }
}