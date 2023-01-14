/* Team 5687 (C)2022 */
package org.frc5687.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * TalonFX wrapper class that uses 254's LazyTalonFX that reduces CAN bus / CPU
 * overhead by skipping duplicate set commands. (By default the Talon flushes the Tx buffer on every
 * set call).
 */
public class OutliersTalon extends TalonFX {
    private final String _name;
    protected double _lastSet = Double.NaN;
    protected TalonFXControlMode _lastControlMode = null;

    public OutliersTalon(int port, String canBus, String name) {
        super(port, canBus);
        this.configFactoryDefault();
        this.set(ControlMode.PercentOutput, 0.0);
        _name = name;
    }

    public double getLastSet() {
        return _lastSet;
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        if (value != _lastSet || mode != _lastControlMode) {
            _lastSet = value;
            _lastControlMode = mode;
            super.set(mode, value);
        }
    }

    public void configure(Configuration config) {
        this.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        this.clearMotionProfileHasUnderrun(config.TIME_OUT);
        this.clearMotionProfileTrajectories();

        this.clearStickyFaults(config.TIME_OUT);

        this.configForwardLimitSwitchSource(
                LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, config.TIME_OUT);
        this.configReverseLimitSwitchSource(
                LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, config.TIME_OUT);
        this.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re-zeroing by default.
        this.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, config.TIME_OUT);
        this.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, config.TIME_OUT);

        this.configNominalOutputForward(0, config.TIME_OUT);
        this.configNominalOutputReverse(0, config.TIME_OUT);
        this.configNeutralDeadband(config.NEUTRAL_DEADBAND, config.TIME_OUT);

        this.configMotorCommutation(MotorCommutation.Trapezoidal);

        this.configPeakOutputForward(1.0, config.TIME_OUT);
        this.configPeakOutputReverse(-1.0, config.TIME_OUT);

        this.setNeutralMode(config.NEUTRAL_MODE);

        this.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, config.TIME_OUT);
        this.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, config.TIME_OUT);

        this.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, config.TIME_OUT);
        this.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, config.TIME_OUT);
        this.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        this.setInverted(config.INVERTED);
        this.setSensorPhase(config.SENSOR_PHASE);

        this.selectProfileSlot(0, 0);

        this.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, config.TIME_OUT);
        this.configVelocityMeasurementWindow(
                config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, config.TIME_OUT);

        this.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, config.TIME_OUT);
        this.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, config.TIME_OUT);

        this.configVoltageCompSaturation(config.VOLTAGE_COMPENSATION, config.TIME_OUT);
        this.configVoltageMeasurementFilter(
                config.VOLTAGE_MEASUREMENT_ROLLING_AVERAGE_WINDOW, config.TIME_OUT);
        this.enableVoltageCompensation(config.ENABLE_VOLTAGE_COMPENSATION);

        this.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        config.ENABLE_SUPPLY_CURRENT_LIMIT,
                        config.SUPPLY_CURRENT_LIMIT,
                        config.SUPPLY_THRESHOLD_CURRENT,
                        config.SUPPLY_THRESHOLD_TIME),
                config.TIME_OUT);
        this.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(
                        config.ENABLE_STATOR_CURRENT_LIMIT,
                        config.STATOR_CURRENT_LIMIT,
                        config.STATOR_THRESHOLD_CURRENT,
                        config.STATOR_THRESHOLD_TIME),
                config.TIME_OUT);

        this.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor, 0, config.TIME_OUT);
        this.configIntegratedSensorInitializationStrategy(
                config.SENSOR_INITIALIZATION_STRATEGY, config.TIME_OUT);
        this.configIntegratedSensorOffset(config.SENSOR_OFFSET_DEGREES, config.TIME_OUT);

        this.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                config.GENERAL_STATUS_FRAME_RATE_MS,
                config.TIME_OUT);
        this.setStatusFramePeriod(
                StatusFrameEnhanced.Status_2_Feedback0,
                config.FEEDBACK_STATUS_FRAME_RATE_MS,
                config.TIME_OUT);
        this.setStatusFramePeriod(
                StatusFrameEnhanced.Status_3_Quadrature,
                config.QUAD_ENCODER_STATUS_FRAME_RATE_MS,
                config.TIME_OUT);
        this.setStatusFramePeriod(
                StatusFrameEnhanced.Status_4_AinTempVbat,
                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,
                config.TIME_OUT);
        this.setStatusFramePeriod(
                StatusFrameEnhanced.Status_8_PulseWidth,
                config.PULSE_WIDTH_STATUS_FRAME_RATE_MS,
                config.TIME_OUT);

        this.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);
    }

    public void configureClosedLoop(ClosedLoopConfiguration config) {
        checkError(
                this.config_kP(config.SLOT, config.kP, config.TIME_OUT),
                "Failed to configure kP on " + _name);
        checkError(
                this.config_kI(config.SLOT, config.kI, config.TIME_OUT),
                "Failed to configure kI on " + _name);
        checkError(
                this.config_kD(config.SLOT, config.kD, config.TIME_OUT),
                "Failed to configure kD on " + _name);
        checkError(
                this.config_kF(config.SLOT, config.kF, config.TIME_OUT),
                "Failed to configure kF on " + _name);
        checkError(
                this.configMaxIntegralAccumulator(
                        config.SLOT, config.MAX_INTEGRAL_ACCUMULATOR, config.TIME_OUT),
                "Failed to configure max integral accumulator on " + _name);
        checkError(
                this.config_IntegralZone(config.SLOT, config.I_ZONE, config.TIME_OUT),
                "Failed to configure integral zone on " + _name);
        checkError(
                this.configAllowableClosedloopError(config.SLOT, config.TOLERANCE, config.TIME_OUT),
                "Failed to configure closed loop error on " + _name);
        checkError(
                this.configClosedloopRamp(config.RAMP_RATE, config.TIME_OUT),
                "Failed to configure ramp rate on " + _name);
        checkError(
                this.configMotionCruiseVelocity(config.CRUISE_VELOCITY, config.TIME_OUT),
                "Failed to configure cruise velocity on " + _name);
        checkError(
                this.configMotionAcceleration(config.ACCELERATION, config.TIME_OUT),
                "Failed to configure acceleration on " + _name);

        this.selectProfileSlot(config.SLOT, 0);
    }

    protected void checkError(ErrorCode error, String message) {
        if (error != ErrorCode.OK) {
            DriverStation.reportError(message + error, false);
        }
    }

    public static double ticksToRadians(double ticks, double gearRatio) {
        return ticks * ((2.0 * Math.PI) / (gearRatio * 2048.0));
    }

    public static double radiansToTicks(double degrees, double gearRatio) {
        return degrees / ((2.0 * Math.PI) / (gearRatio * 2048.0));
    }

    public static double ticksPer100msToRPM(double velocityCounts, double gearRatio) {
        double RPM = velocityCounts * (600.0 / 2048.0);
        return RPM / gearRatio;
    }

    public static double RPMToTicksPer100ms(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        return motorRPM * (2048.0 / 600.0);
    }

    public static class ClosedLoopConfiguration {
        public int TIME_OUT = 100;
        public int SLOT = 0;

        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.0;
        public double MAX_INTEGRAL_ACCUMULATOR = 0;
        public int I_ZONE = 0; // Ticks
        public int TOLERANCE = 0; // Ticks

        public double RAMP_RATE = 0.0;

        public int CRUISE_VELOCITY = 0; // ticks per 100ms
        public int ACCELERATION = 0; // ticks per 100ms
    }

    public static class Configuration {
        public int TIME_OUT = 100; // ms
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        // factory default
        public double NEUTRAL_DEADBAND = 0.04;

        public SensorInitializationStrategy SENSOR_INITIALIZATION_STRATEGY =
                SensorInitializationStrategy.BootToZero;
        public double SENSOR_OFFSET_DEGREES = 0;

        public double VOLTAGE_COMPENSATION = 0.0;
        public int VOLTAGE_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 32;
        public boolean ENABLE_VOLTAGE_COMPENSATION = false;

        public boolean ENABLE_SUPPLY_CURRENT_LIMIT = false;
        public double SUPPLY_CURRENT_LIMIT = 20;
        public double SUPPLY_THRESHOLD_CURRENT = 60;
        public double SUPPLY_THRESHOLD_TIME = 0.2;
        public boolean ENABLE_STATOR_CURRENT_LIMIT = false;
        public double STATOR_CURRENT_LIMIT = 20;
        public double STATOR_THRESHOLD_CURRENT = 60;
        public double STATOR_THRESHOLD_TIME = 0.2;

        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int FORWARD_SOFT_LIMIT = 0;
        public int REVERSE_SOFT_LIMIT = 0;

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;

        public int CONTROL_FRAME_PERIOD_MS = 10;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        public int GENERAL_STATUS_FRAME_RATE_MS = 10;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD =
                SensorVelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;
    }
}
