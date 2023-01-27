/* Team 5687 (C)2020-2022 */
package org.frc5687.chargedup;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.chargedup.subsystems.DiffSwerveModule;

public class Constants {
    public static final int TICKS_PER_UPDATE = 5;
    public static final double METRIC_FLUSH_PERIOD = 2.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static final double CONTROL_PERIOD = 0.005; // 5 ms
    public static final double DATA_PERIOD = 0.02; // 20 ms
    public static final double EPSILON = 1e-12;
    
    /**
     * Coordinate System
     *
     * <p>(X, Y): X is N or S, N is + Y is W or E, W is +
     *
     * <p>NW (+,+) NE (+,-)
     *
     * <p>SW (-,+) SE (-,-)
     *
     * <p>We go counter-counter clockwise starting at NW of chassis:
     *
     * <p>NW, SW, SE, NE
     *
     * <p>Note: when robot is flipped over, this is clockwise.
     */
    public static class DriveTrain {
        public static final String CAN_BUS = "Tomcat";
    
        // Size of the robot chassis in meters
        public static final double WIDTH = 0.4191; // meters
        public static final double LENGTH = 0.4191; // meters
        // Distance of swerve modules from center of robot
        public static final double SWERVE_NS_POS = LENGTH / 2.0;
        public static final double SWERVE_WE_POS = WIDTH / 2.0;
        public static final DiffSwerveModule.ModuleConfiguration NORTH_WEST_CONFIG = new DiffSwerveModule.ModuleConfiguration();
        static {
            NORTH_WEST_CONFIG.moduleName = "North West";
            NORTH_WEST_CONFIG.canBus = CAN_BUS;
            NORTH_WEST_CONFIG.position = new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS); // +,+
    
            NORTH_WEST_CONFIG.encoderInverted = false;
            NORTH_WEST_CONFIG.encoderOffset = -0.0305;
        }
        public static final DiffSwerveModule.ModuleConfiguration SOUTH_WEST_CONFIG = new DiffSwerveModule.ModuleConfiguration();
        static {
            SOUTH_WEST_CONFIG.moduleName = "South West";
            SOUTH_WEST_CONFIG.canBus = CAN_BUS;
            SOUTH_WEST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS); // -,+
    
            SOUTH_WEST_CONFIG.encoderInverted = false;
            SOUTH_WEST_CONFIG.encoderOffset = 0.0;
        }
        public static final DiffSwerveModule.ModuleConfiguration SOUTH_EAST_CONFIG = new DiffSwerveModule.ModuleConfiguration();
        static {
            SOUTH_EAST_CONFIG.moduleName = "South East";
            SOUTH_EAST_CONFIG.canBus = CAN_BUS;
            SOUTH_EAST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, -SWERVE_WE_POS); // -,-
    
            SOUTH_EAST_CONFIG.encoderInverted = false;
            SOUTH_EAST_CONFIG.encoderOffset = 0.0;
        }
        public static final DiffSwerveModule.ModuleConfiguration NORTH_EAST_CONFIG = new DiffSwerveModule.ModuleConfiguration();
        static {
            NORTH_EAST_CONFIG.moduleName = "North East";
            NORTH_EAST_CONFIG.canBus = CAN_BUS;
            NORTH_EAST_CONFIG.position = new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS); // +,-
    
            NORTH_EAST_CONFIG.encoderInverted = false;
            NORTH_EAST_CONFIG.encoderOffset = -0.205;
        }
        public static final double kDt = 0.005; // 5 ms
        public static final double TRANSLATION_DEADBAND =
                0.1; // Avoid unintentional joystick movement
        public static final double ROTATION_DEADBAND = 0.1; // Avoid unintentional joystick movement
        public static final double TRANSLATION_POWER = 1.75; // Determines the curve of drive input
        public static final double ROTATION_POWER = 1.0; // Determines the curve of drive input
        public static final long DISABLE_TIME = 500; // ms
    
        public static final double LINEAR_VELOCITY_REFERENCE = 0.5;
    
        // Maximum rates of motion
        public static final double MAX_MPS = 3.0; // Max speed of robot (m/s)
        public static final double MAX_AUTO_MPS = 2.0; // Max speed of robot (m/s)
        public static final double MAX_MPS_DURING_CLIMB =
                MAX_MPS / 4; // Max speed of robot (m/s) during climb
        public static final double MAX_ANG_VEL =
                Math.PI * 1.5; // Max rotation rate of robot (rads/s)
        public static final double MAX_MPSS = 1; // Max acceleration of robot (m/s^2)
    
        public static final double POLE_THRESHOLD = Units.degreesToRadians(5.0);
    
        // PID controller settings
        public static final double STABILIZATION_kP = 1;
        public static final double STABILIZATION_kI = 0.0;
        public static final double STABILIZATION_kD = 0.0;
    
        public static final double SNAP_kP = 4.0;
        public static final double SNAP_kI = 0.0;
        public static final double SNAP_kD = 0.0;
    
        public static final double VISION_kP = 4.0;
        public static final double VISION_kI = 0.0;
        public static final double VISION_kD = 0.0;
    
        public static final double PROFILE_CONSTRAINT_VEL = MAX_ANG_VEL;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI * 3.0;
    
        public static final double kP = 5;
        public static final double kI = 0.0;
        public static final double kD = 0.5;
    
        public static final double POSITION_TOLERANCE = 0.01;
    }
    
    public static class DifferentialSwerveModule {
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();
        // this is the motor config for the diff swerve motors
        static {
            CONFIG.TIME_OUT = 100;
    
            CONFIG.NEUTRAL_MODE = NeutralMode.Brake;
            CONFIG.INVERTED = false;
    
            CONFIG.VOLTAGE_COMPENSATION = 12.0;
            CONFIG.ENABLE_VOLTAGE_COMPENSATION = true;
    
            CONFIG.VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_1Ms;
            CONFIG.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 32;
    
            CONFIG.STATOR_CURRENT_LIMIT = 60;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
    
            CONFIG.FEEDBACK_STATUS_FRAME_RATE_MS = 5;
            CONFIG.GENERAL_STATUS_FRAME_RATE_MS = 20;
        }
        // update rate of our modules 5ms.
        public static final double kDt = 0.005;
    
        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6380);
        public static final double GEAR_RATIO_WHEEL = 6.46875 / 1.2;
        public static final double GEAR_RATIO_STEER = 9.2 / 1.2;
        public static final double FALCON_RATE = 600.0;
        public static final double WHEEL_RADIUS = 0.04615; // Meters with compression.
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTAGE = 12.0;
    
        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.005;
        public static final double INERTIA_STEER = 0.004;
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.
        public static final double Q_AZIMUTH_ANG_VELOCITY = 1.1; // radians per sec
        public static final double Q_AZIMUTH = 0.08; // radians
        public static final double Q_WHEEL_ANG_VELOCITY = 1; // radians per sec
        // This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
        // Model noise are assuming that our model isn't as accurate as our sensors.
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = .1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty, so we
        // increase the noise.
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double CONTROL_EFFORT = VOLTAGE;
    
        public static final double MAX_MODULE_SPEED_MPS =
                (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * WHEEL_RADIUS;
        public static final double MAX_ANGULAR_VELOCITY = FALCON_FREE_SPEED / GEAR_RATIO_STEER;
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_VELOCITY * 20;
    
        public static final double MAX_MODULE_ACCELERATION =
                (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * 20;
        public static final double MAX_MODULE_JERK = MAX_MODULE_ACCELERATION * 10;
    }

    public static class Arm {
        public static final double kDt = 0.02;
        public static final String CAN_BUS = "rio";
        public static final double GEAR_RATIO = 125;
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();
        static {
            CONFIG.TIME_OUT = 100;

            CONFIG.NEUTRAL_MODE = NeutralMode.Brake;
            CONFIG.INVERTED = false;

            CONFIG.VOLTAGE_COMPENSATION = 12.0;
            CONFIG.ENABLE_VOLTAGE_COMPENSATION = true;

            CONFIG.STATOR_CURRENT_LIMIT = 60;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;

            CONFIG.FEEDBACK_STATUS_FRAME_RATE_MS = 10;
            CONFIG.GENERAL_STATUS_FRAME_RATE_MS = 20;
        }
        // this is the motor config for the arm motors

        // Physical characteristic of the system
        public static final double INERTIA_ARM = 0.5; // kg * m^2

        // Kalman Filter parameters
        public static final double MODEL_POSITION_NOISE = 0.015; // m
        public static final double MODEL_VELOCITY_NOISE = 0.16; // m/s

        public static final double SENSOR_POSITION_NOISE = 0.01; // m
        // LQR parameters
        public static final double Q_POSITION = Units.degreesToRadians(1.0);
        public static final double Q_VELOCITY = Units.degreesToRadians(10.0);

        public static final double CONTROL_EFFORT = 12.0;
        // profile constraints
        public static final double MAX_VELOCITY = Units.degreesToRadians(45);
        public static final double MAX_ACCELERATION = Units.degreesToRadians(90);

    }

    public static class Auto {
        public static class FieldPoses {
            public static final Pose2d POSE_1 = new Pose2d(0, 0, new Rotation2d());
            public static final Pose2d POSE_2 = new Pose2d(1, 1, new Rotation2d(Math.PI / 2));
            public static final Pose2d POSE_3 = new Pose2d(2, 2, new Rotation2d());

        }
    

        public static class TrajectoryPoints {
            public static class S {
                public static final List<Pose2d> waypoints =
                        Arrays.asList(
                                FieldPoses.POSE_1,
                                FieldPoses.POSE_2,
                                FieldPoses.POSE_3);
            }
        }
    }
}

