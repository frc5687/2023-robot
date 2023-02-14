/* Team 5687 (C)5687-2022 */
package org.frc5687.chargedup.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.math.*;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.util.Helpers;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.math.LinearSystems;

import static org.frc5687.chargedup.Constants.DifferentialSwerveModule.*;

/**
 * Created 10/11/2020 by Dennis Slobodzian.
 *
 * <p>Wrapper class for a differential swerve module using LQR as the controller for azimuth angle
 * and wheel velocity of the module.
 */
public class DiffSwerveModuleCurrent {
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
    private final String _name;

    private TrapezoidProfile.State _angleSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State _angleGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State _wheelVelocityReference = new TrapezoidProfile.State();
    private final TrapezoidProfile.Constraints _profiledSteerConstraints;
    private final TrapezoidProfile.Constraints _profiledWheelConstraints;
    public DiffSwerveModuleCurrent(
            DiffSwerveModule.ModuleConfiguration config, int leftMotorID, int rightMotorID, int encoderPort) {
        // setup azimuth bore encoder.
        _name = config.moduleName;
        _boreEncoder = new DutyCycleEncoder(encoderPort);
        _boreEncoder.setDistancePerRotation(2.0 * Math.PI);

        _encoderOffset = config.encoderOffset;
        _encoderInverted = config.encoderInverted;

        _reference = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);
        _positionVector = config.position;
        // setup both falcon motors.
        _leftFalcon = new OutliersTalon(leftMotorID, config.canBus, (config.moduleName + " Left"));
        _rightFalcon = new OutliersTalon(rightMotorID, config.canBus, (config.moduleName + " Right"));

        _leftFalcon.configure(CONFIG);
        _rightFalcon.configure(CONFIG);
        _leftFalcon.setTorqueCurrentFOCRate(0);
        _leftFalcon.setTorqueCurrentFOCRate(0);

//        _leftFalcon.configureClosedLoop(CLOSED_LOOP_CONFIGURATION);
//        _rightFalcon.configureClosedLoop(CLOSED_LOOP_CONFIGURATION);

        // Creates a Linear System of our Differential Swerve Module.
        LinearSystem<N3, N2, N3> swerveModuleModel =
                LinearSystems.createDifferentialSwerveModuleCurrent(
                        LinearSystems.getFalcon500FOC(2),
                        INERTIA_STEER,
                        INERTIA_WHEEL,
                        GEAR_RATIO_STEER,
                        GEAR_RATIO_WHEEL,
                        FRICTION_STEER,
                        FRICTION_WHEEL
                );

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

        moduleController.latencyCompensate(swerveModuleModel, 0.005, 0.005);
        // Creates a LinearSystemLoop that contains the Model, Controller, Observer, Max Volts,
        // Update Rate.
        Matrix<N2, N1> u_limit = VecBuilder.fill(CONFIG.MAX_CURRENT,CONFIG.MAX_CURRENT);
        _moduleControlLoop =
                new LinearSystemLoop<>(
                        swerveModuleModel,
                        moduleController,
                        moduleObserver,
                        u -> StateSpaceUtil.clampInputMaxMagnitude(
                                u,
                                u_limit.times(-1.0),
                                u_limit),
                        kDt
                );
        //         Initializes the vectors and matrices.
        System.out.println("K mat:\n" + _moduleControlLoop.getController().getK().toString());

        _moduleControlLoop.reset(VecBuilder.fill(0, 0, 0));

        _leftFalcon.getVelocity().setUpdateFrequency(200);
        _leftFalcon.getPosition().setUpdateFrequency(200);
        _rightFalcon.getVelocity().setUpdateFrequency(200);
        _rightFalcon.getPosition().setUpdateFrequency(200);

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

    public ControlState getControlState() {
        return _controlState;
    }

    public void setControlState(ControlState state) {
        _controlState = state;
    }

    public synchronized void readInputs() {
        _systemIO.leftVelocityRotationsPerSec = _leftFalcon.getVelocity().getValue();
        _systemIO.leftPositionRotations = _leftFalcon.getPosition().getValue();
        _systemIO.rightVelocityRotationsPerSec = _rightFalcon.getVelocity().getValue();
        _systemIO.rightPositionRotations = _rightFalcon.getPosition().getValue();

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
//                Matrix<N3, N1> ref = VecBuilder.fill(Math.PI, 0.0, 20);
                _moduleControlLoop.setNextR(_reference);
                // updates the kalman filter with new data points.
                _moduleControlLoop.correct(
                        VecBuilder.fill(
                                getModuleAngle(),
                                getAzimuthAngularVelocity(),
                                getWheelAngularVelocity()));
                predict();

                setLeftFalconCurrent(getLeftNextCurrent());
                setRightFalconCurrent(getRightNextCurrent());
//                setLeftFalconCurrent(10);
                break;
        }
    }
    private Matrix<N3, N1> profiledReference(Matrix<N3, N1> reference, Matrix<N3, N1> xHat) {
        double errorBound = (Math.PI - (-Math.PI)) / 2.0;
        _angleGoal = new TrapezoidProfile.State(reference.get(0, 0), 0);
        double angleMinimumGoalDistance = MathUtil.inputModulus(
                _angleGoal.position - getModuleAngle(),
                -errorBound,
                errorBound
        );
        double angleMinimumSetpointDistance = MathUtil.inputModulus(
                _angleSetpoint.position - getModuleAngle(),
                -errorBound,
                errorBound
        );
        _angleGoal.position = angleMinimumGoalDistance + getModuleAngle();
        _angleSetpoint.position = angleMinimumSetpointDistance + getModuleAngle();
//        _angleSetpoint.position = getModuleAngle();
//        _angleSetpoint.velocity= getModuleAngle();

        var steerProfile = new TrapezoidProfile(
                _profiledSteerConstraints,
                _angleGoal,
                _angleSetpoint);

        _angleSetpoint = steerProfile.calculate(Constants.DifferentialSwerveModule.kDt);
        var wheelProfile = new TrapezoidProfile(
                _profiledWheelConstraints,
                new TrapezoidProfile.State(reference.get(2,0),0),
                _wheelVelocityReference
        );
        _wheelVelocityReference = wheelProfile.calculate(Constants.DifferentialSwerveModule.kDt);

        return VecBuilder.fill(
                _angleSetpoint.position - getModuleAngle(),
                _angleSetpoint.velocity - getAzimuthAngularVelocity(),
//                error.get(2,0));
//                0.0);
                _wheelVelocityReference.position - getWheelAngularVelocity());
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
//        Matrix<N3, N1> error = reference.minus(xHat);
        double aziVelError = reference.get(1, 0) - getAzimuthAngularVelocity();
        double wheelVelError = reference.get(2, 0) - getWheelAngularVelocity();
//        return VecBuilder.fill(positionError, error.get(1, 0), error.get(2, 0));
        return VecBuilder.fill(positionError, aziVelError, wheelVelError);
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
                                .times(
//                                        profiledReference(
//                                                _moduleControlLoop.getNextR(),
//                                                _moduleControlLoop.getXHat()
//                                        )));
                                        wrapAngle(
                                                _moduleControlLoop.getNextR(),
                                                _moduleControlLoop.getXHat())));
//                                .plus(
//                                        _moduleControlLoop
//                                                .getFeedforward()
//                                                .calculate(_moduleControlLoop.getNextR())));
        _moduleControlLoop.getObserver().predict(_u, kDt);
    }

    public void start() {
        setControlState(ControlState.STATE_CONTROL);
    }

    public void stop() {
    }

    // location is x, y position w.r.t robot frame
    public Translation2d getModuleLocation() { return _positionVector; }
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getWheelDistance(), new Rotation2d(getModuleAngle()));
    }

    public void setRightFalconCurrent(double current) {
//        _rightFalcon.set(TalonFXControlMode.Current, current);
        _rightFalcon.setTorqueCurrentFOC(current);
    }
    public void setRightFalconVoltage(double voltage) {
        double limVoltage = Helpers.limit(voltage, -VOLTAGE, VOLTAGE);
        _rightFalcon.setVoltage(limVoltage);
    }

    public void setLeftFalconVoltage(double voltage) {
        double limVoltage = Helpers.limit(voltage, -VOLTAGE, VOLTAGE);
        _leftFalcon.setVoltage(limVoltage);
    }
    public void setLeftFalconCurrent(double current) {
        _leftFalcon.setTorqueCurrentFOC(current);
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
        return OutliersTalon.rotationsPerSecToRPM(_systemIO.rightVelocityRotationsPerSec, 1.0);
        // return OutliersTalon.ticksPer100msToRPM(_systemIO.rightVelocityTicksPer100ms, 1.0);
    }
    public double getRightFalconDistanceRadians() {
        return _systemIO.rightPositionRotations * (Math.PI * 2.0);

        // return OutliersTalon.ticksToRadians(_systemIO.rightPositionTicks, 1.0);
    }
    public double getLeftFalconRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_systemIO.leftVelocityRotationsPerSec, 1.0);

        // return OutliersTalon.ticksPer100msToRPM(_systemIO.leftVelocityTicksPer100ms, 1.0);
    }
    public double getLeftFalconDistanceRadians() {
        return _systemIO.leftPositionRotations * (Math.PI * 2.0);
        // return OutliersTalon.ticksToRadians(_systemIO.leftPositionTicks, 1.0);
    }

    public void resetEncoders() {
        _leftFalcon.setRotorPosition(0);
        _rightFalcon.setRotorPosition(0);
    }

    public double getLeftVoltage() {
        return _leftFalcon.getSupplyVoltage().getValue();
    }

    public double getRightVoltage() {
        return _rightFalcon.getSupplyVoltage().getValue();
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
    public double getLeftNextCurrent() {
        return _u.get(0, 0);
    }

    public double getRightNextCurrent() {
        return _u.get(1, 0);
    }

    public double getLeftCurrent() {
        return _leftFalcon.getStatorCurrent().getValue();
    }

    public double getRightCurrent() {
        return _rightFalcon.getStatorCurrent().getValue();
    }

    public double getReferenceModuleAngle() {
        return _moduleControlLoop.getNextR(0);
    }

    public double getReferenceModuleAngularVelocity() {
        return _moduleControlLoop.getNextR(1);
    }

    public double getReferenceWheelVelocity() {
        return _moduleControlLoop.getNextR(2) * WHEEL_RADIUS;
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
        setModuleState(SwerveModuleState.optimize(state, new Rotation2d(getModuleAngle())));
    }

    public void updateDashboard() {
//        SmartDashboard.putNumber(_name + "/leftVoltage", _leftFalcon.getMotorOutputVoltage());
//        SmartDashboard.putNumber(_name + "/rightVoltage", _rightFalcon.getMotorOutputVoltage());
//        SmartDashboard.putNumber(_name + "/leftNextCurrent", getLeftNextCurrent());
//        SmartDashboard.putNumber(_name + "/rightNextCurrent", getRightNextCurrent());
//        SmartDashboard.putNumber(_name + "/leftSupplyCurrent", _leftFalcon.getSupplyCurrent());
//        SmartDashboard.putNumber(_name + "/rightSupplyCurrent", _rightFalcon.getSupplyCurrent());
//        SmartDashboard.putNumber(_name + "/leftStatorCurrent", getLeftCurrent());
//        SmartDashboard.putNumber(_name + "/rightStatorCurrent", getRightCurrent());
        SmartDashboard.putNumber(_name + "/referenceAngle", getReferenceModuleAngle());
        SmartDashboard.putNumber(_name + "/moduleAngle", getModuleAngle());
        SmartDashboard.putNumber(_name + "/velocityWheel", getWheelVelocity());
        SmartDashboard.putNumber(_name + "/referenceWheelVelocity", getReferenceWheelVelocity());
//
//        SmartDashboard.putString(_name + "/KMatrix", _moduleControlLoop.getController().getK().toString());
//
//        SmartDashboard.putNumber(_name + "/estimatedModuleAngle", getPredictedAzimuthAngle());
//        SmartDashboard.putString(_name + "/refernce", _reference.toString());
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
        public double leftVelocityRotationsPerSec;
        public double leftPositionRotations;
        public double rightVelocityRotationsPerSec;
        public double rightPositionRotations;
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