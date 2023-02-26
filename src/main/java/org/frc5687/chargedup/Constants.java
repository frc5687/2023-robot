/* Team 5687 (C)2020-2022 */
package org.frc5687.chargedup;

import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.List;
import org.frc5687.chargedup.subsystems.DiffSwerveModule;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;

public class Constants {
    public static final int TICKS_PER_UPDATE = 10;
    public static final double METRIC_FLUSH_PERIOD = 2.0;
    public static final double UPDATE_PERIOD = 0.02; // 20 ms
    public static final double CONTROL_PERIOD = 0.005; // 10 ms
    public static final double DATA_PERIOD = 0.02; // 20 ms
    public static final double EPSILON = 1e-9;

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
        public static final String CAN_BUS = "CANivore";
        public static final int NUM_MODULES = 4;

        // Size of the robot chassis in meters
        public static final double WIDTH = 0.4445; // meters
        public static final double LENGTH = 0.4445; // meters
        // Distance of swerve modules from center of robot
        public static final double SWERVE_NS_POS = LENGTH / 2.0;
        public static final double SWERVE_WE_POS = WIDTH / 2.0;

        public static final double MAX_MPS = 4.0; // Max speed of robot (m/s)
        public static final double SLOW_MPS = 2.0; // Slow speed of robot (m/s)
        public static final double MAX_ANG_VEL = Math.PI * 2.0; // Max rotation rate of robot (rads/s)
        public static final double SLOW_ANG_VEL = Math.PI; // Max rotation rate of robot (rads/s)
        public static final KinematicLimits KINEMATIC_LIMITS = new KinematicLimits();

        static {
            KINEMATIC_LIMITS.maxDriveVelocity = 5.3; // m/s
            KINEMATIC_LIMITS.maxDriveAcceleration = 25; // m/s^2
            KINEMATIC_LIMITS.maxSteeringVelocity = 20; // rad/s
        }

        public static final DiffSwerveModule.ModuleConfiguration NORTH_WEST_CONFIG =
                new DiffSwerveModule.ModuleConfiguration();

        static {
            NORTH_WEST_CONFIG.moduleName = "North West";
            NORTH_WEST_CONFIG.canBus = CAN_BUS;
            NORTH_WEST_CONFIG.position = new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS); // +,+

            NORTH_WEST_CONFIG.encoderInverted = false;
            NORTH_WEST_CONFIG.encoderOffset = -0.085;
        }

        public static final DiffSwerveModule.ModuleConfiguration SOUTH_WEST_CONFIG =
                new DiffSwerveModule.ModuleConfiguration();

        static {
            SOUTH_WEST_CONFIG.moduleName = "South West";
            SOUTH_WEST_CONFIG.canBus = CAN_BUS;
            SOUTH_WEST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS); // -,+

            SOUTH_WEST_CONFIG.encoderInverted = false;
            SOUTH_WEST_CONFIG.encoderOffset = -0.16;
        }

        public static final DiffSwerveModule.ModuleConfiguration SOUTH_EAST_CONFIG =
                new DiffSwerveModule.ModuleConfiguration();

        static {
            SOUTH_EAST_CONFIG.moduleName = "South East";
            SOUTH_EAST_CONFIG.canBus = CAN_BUS;
            SOUTH_EAST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, -SWERVE_WE_POS); // -,-

            SOUTH_EAST_CONFIG.encoderInverted = false;
            SOUTH_EAST_CONFIG.encoderOffset = -0.08;
        }

        public static final DiffSwerveModule.ModuleConfiguration NORTH_EAST_CONFIG =
                new DiffSwerveModule.ModuleConfiguration();

        static {
            NORTH_EAST_CONFIG.moduleName = "North East";
            NORTH_EAST_CONFIG.canBus = CAN_BUS;
            NORTH_EAST_CONFIG.position = new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS); // +,-

            NORTH_EAST_CONFIG.encoderInverted = false;
            NORTH_EAST_CONFIG.encoderOffset = -0.079;
        }

        public static final double TRANSLATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final double ROTATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final long DISABLE_TIME = 500; // ms

        public static final double LINEAR_VELOCITY_REFERENCE = 0.5;

        // Maximum rates of motion

        public static final double MAX_AUTO_MPS = 2.0; // Max speed of robot (m/s)
        public static final double MAX_MPSS = 1; // Max acceleration of robot (m/s^2)

        public static final double POLE_THRESHOLD = Units.degreesToRadians(5.0);

        // PID controller settings
        public static final double MAINTAIN_kP = 3.0;
        public static final double MAINTAIN_kI = 0.0;
        public static final double MAINTAIN_kD = 0.1;

        public static final double SNAP_kP = 3.5;
        public static final double SNAP_kI = 0.0;
        public static final double SNAP_kD = 0.3;

        public static final double PROFILE_CONSTRAINT_VEL = Math.PI * 4.0;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI * 8.0;

        public static final double kP = 1.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double POSITION_TOLERANCE = 0.01;
        public static final double LEVEL_TOLERANCE = 0.5;
        public static final double HEADING_TOLERANCE = 0.15; // rad
        public static final double BUMP_DEGREES = 10;

        public static final double PITCH_LOOKING_ANGLE =
                Units.degreesToRadians(15.0); // this is degrees because sad.
        public static final double PITCH_LEVELED_ANGLE =
                Units.degreesToRadians(5.0); // this is degrees because sad.

        public static final double DRIVING_UP_RAMP_SPEEDS_VX = 2.0;
        public static final double DRIVING_DOWN_RAMP_SPEEDS_VX = 1.0;
    }

    public static class DifferentialSwerveModule {
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();
        // this is the motor config for the diff swerve motors
        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_STATOR_CURRENT = 80;
            CONFIG.MAX_CURRENT = 80;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
            //            CONFIG.USE_FOC = true;
        }

        public static final OutliersTalon.ClosedLoopConfiguration CLOSED_LOOP_CONFIGURATION =
                new OutliersTalon.ClosedLoopConfiguration();

        //         update rate of our modules 5ms.
        public static final double kDt = 0.005;
        //        public static final double kDt = 0.01;
        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6080); // was 6380 foc is different speed
        public static final double GEAR_RATIO_WHEEL = 6.46875 / 1.2;
        public static final double GEAR_RATIO_STEER = 9.2 / 1.2;

        public static final double FRICTION_STEER = 0.00;
        public static final double FRICTION_WHEEL = 0.00;
        public static final double WHEEL_RADIUS = 0.04615; // Meters with compression.
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTAGE = 12.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_STEER = 0.005;
        public static final double INERTIA_WHEEL = 0.003;
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.

        public static final double Q_AZIMUTH = 0.08; // radians
        public static final double Q_AZIMUTH_ANG_VELOCITY = 10.0; // radians per sec
        public static final double Q_WHEEL_ANG_VELOCITY = 1.0; // radians per sec

        public static final double CONTROL_EFFORT = 4.0;
        // This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
        // Model noise are assuming that our model isn't as accurate as our sensors.
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = 0.1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 1.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 1.0; // radians per sec
        // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty, so we
        // increase the noise.
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double MAX_MODULE_SPEED_MPS =
                (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * WHEEL_RADIUS;
        public static final double MAX_ANGULAR_VELOCITY = FALCON_FREE_SPEED / GEAR_RATIO_STEER;
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_VELOCITY * 5;

        public static final double MAX_MODULE_ACCELERATION = (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * 4;
        public static final double MAX_MODULE_JERK = MAX_MODULE_ACCELERATION * 2;
    }

    public static class ExtendingArm {
        public static final String CAN_BUS = "CANivore";
        public static final double GEAR_RATIO = 25;
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();

        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            CONFIG.INVERTED = InvertedValue.Clockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_STATOR_CURRENT = 40;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
            CONFIG.USE_FOC = true;
        }

        public static final double SHORT_ARM_DISTANCE = .2;
        public static final double MEDIUM_ARM_DISTANCE = .4;
        public static final double LONG_ARM_DISTANCE = .6;

        public static final double ZERO_ARM_SPEED = 0;
        public static final double ZERO_ENCODER = 0.0;

        public static final double OUT_HALL_ENCODER_ROTATIONS = 120;
        public static final double IN_HALL_RAD = 0;

        public static final double kP = 2.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double EXT_ARM_TOLERANCE = .01;

        public static final double ROTATIONS_TO_METERS = 411281.3171 / 2048;

        public static final OutliersTalon.ClosedLoopConfiguration CONTROLLER_CONFIG =
                new OutliersTalon.ClosedLoopConfiguration();

        static {
            CONTROLLER_CONFIG.SLOT = 0;

            CONTROLLER_CONFIG.kP = 1.3;
            CONTROLLER_CONFIG.kI = 0;
            CONTROLLER_CONFIG.kD = 0.0;
            CONTROLLER_CONFIG.kF = 0;

            CONTROLLER_CONFIG.CRUISE_VELOCITY = 100;
            CONTROLLER_CONFIG.ACCELERATION = 600;
            CONTROLLER_CONFIG.JERK = 3200;
        }
    }

    public static class Arm {
        public static final double kDt = 0.02;
        public static double MOTOR_kT = DCMotor.getFalcon500(1).KtNMPerAmp;
        public static double MOTOR_R = DCMotor.getFalcon500(1).rOhms;
        public static final String CAN_BUS = "CANivore";
        public static final double GEAR_RATIO = 375;
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();

        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            CONFIG.INVERTED = InvertedValue.Clockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_STATOR_CURRENT = 60;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
            CONFIG.USE_FOC = true;
        }
        // this is the motor config for the arm motors

        public static final double ARM_LENGTH = Units.feetToMeters(4.0);
        public static final double ARM_WEIGHT = Units.lbsToKilograms(10.0);
        public static final double INERTIA_ARM = (1.0 / 3.0) * ARM_WEIGHT * (ARM_LENGTH * ARM_LENGTH);

        // Physical characteristic of the system
        //        public static final double INERTIA_ARM = 0.5; // kg * m^2

        // Kalman Filter parameters
        public static final double MODEL_POSITION_NOISE = 0.015; // rad
        public static final double MODEL_VELOCITY_NOISE = 0.04; // rad/s

        public static final double SENSOR_POSITION_NOISE = 0.01; // rad
        // LQR parameters
        public static final double Q_POSITION = Units.degreesToRadians(0.1);
        public static final double Q_VELOCITY = Units.degreesToRadians(1);

        public static final double CONTROL_EFFORT = 12.0;
        // profile constraints
        public static final double MAX_VELOCITY = Units.degreesToRadians(100);
        public static final double MAX_ACCELERATION = Units.degreesToRadians(100);

        public static final double ANGLE_TOLERANCE = 0.05; // rads
        public static final double VERTICAL_ARM_ANGLE = 1.22; // rads
        public static final double LOWER_EXTREME = 0.378;
        public static final double PLACE_ARM_ANGLE = 0.255; // testing
    }

    public static class Auto {
        public static class FieldPoses {
            public static final Pose2d POSE_1 = new Pose2d(0, 0, new Rotation2d());
            public static final Pose2d POSE_2 = new Pose2d(1, 1, new Rotation2d(Math.PI / 2));
            public static final Pose2d POSE_3 = new Pose2d(2, 2, new Rotation2d());
        }

        public static final Pose2d STARTING_ONE = new Pose2d(1.820, 3.04, new Rotation2d());
        public static final Pose2d STARTING_CHARGING_STATION =
                new Pose2d(1.820, 4.025, new Rotation2d());
        public static final Pose2d STARTING_ONE_TEMP = new Pose2d(0, 0, new Rotation2d());
        public static final Pose2d TARGET_ONE = new Pose2d(7.065, 3.456, new Rotation2d(Math.PI / 2.0));
        public static final Pose2d TARGET_TWO = new Pose2d(7.065, 4.676, new Rotation2d());
        public static final Pose2d TARGET_THREE = new Pose2d(7.065, 5.844, new Rotation2d());
        public static final Pose2d TARGET_FOUR = new Pose2d(7.065, 7.114, new Rotation2d());

        public static class TrajectoryPoints {
            public static class S {
                public static final List<Pose2d> waypoints =
                        Arrays.asList(FieldPoses.POSE_1, FieldPoses.POSE_2, FieldPoses.POSE_3);
            }

            public static class FIRST_TO_TARGET_ONE {
                public static final List<Pose2d> waypoints = Arrays.asList(STARTING_ONE, TARGET_ONE);
            }
        }
    }

    public static class EndEffector {
        public static final double WRIST_OFFSET = 0;
        public static final double GRIPPER_OFFSET = 0;

        public static final double WRIST_kP = 2.2;
        public static final double WRIST_kI = 0;
        public static final double WRIST_kD = 0.03;

        public static final double WRIST_VEL = Units.degreesToRadians(5);
        public static final double WRIST_ACCEL = Units.degreesToRadians(1);

        public static final double WRIST_TOLERENCE = Units.degreesToRadians(3.0);
        public static final double WRIST_MAX_ANGLE = Units.degreesToRadians(320.0);
        public static final double WRIST_MID_ANGLE = Units.degreesToRadians(220);
        public static final double WRIST_MIN_ANGLE = Units.degreesToRadians(120.5);

        public static final double WRIST_SAFE_ANGLE = Units.degreesToRadians(240);
        public static final double WRIST_PICKUP_ANGLE = Units.degreesToRadians(320);
        public static final boolean WRIST_INVERTED = true;

        public static final double GRIPPER_kP = 3.2;
        public static final double GRIPPER_kI = 0;
        public static final double GRIPPER_kD = 0;

        public static final double GRIPPER_STALL_CURRENT = 15.1; // was 10

        public static final double GRIPPER_I_ZONE = 1;

        public static final double GRIPPER_VEL = Units.degreesToRadians(5);
        public static final double GRIPPER_ACCEL = Units.degreesToRadians(1);

        public static final double GRIPPER_TOLERENCE = Units.degreesToRadians(1);
        // fully closed angle
        public static final double GRIPPER_IN_SPEED = -0.5;
        // fully open angle
        public static final double GRIPPER_OUT_SPEED = 1.0;
        // public static final double GRIPPER_CUBE_ANGLE = Units.degreesToRadians(186.0);
        public static final boolean GRIPPPER_INVERTED = false;
        public static final double ROLLER_CUBE_IDLE_SPEED = 0.15;
        public static final double ROLLER_CONE_IDLE_SPEED = -0.25;
        public static final double PLACE_CUBE_ROLLER_SPEED = -1.0;
        public static final double PLACE_CONE_ROLLER_SPEED = 1.0;
        public static final long GRIPPER_TIMEOUT = 1000;
    }

    public static class Vision {
        public static final float Z_CAM_Z_OFFSET = 0.78111f;
        public static final float Z_CAM_Y_OFFSET = 0.17653f;
        public static final float Z_CAM_X_OFFSET = 0.17439f;
    }

    public static class CANdle {
        public static double BRIGHTNESS = 1.0;
        public static int NUM_LED = 128;
        public static double SPEED = 0.1;
        public static TwinklePercent TWINKLEPERCENT = TwinklePercent.Percent42;
        public static TwinkleOffPercent TWINKLEOFFPERCENT = TwinkleOffPercent.Percent42;

        public static int[] YELLOW = {255, 65, 0};
        public static int[] RED = {255, 0, 0};
        public static int[] GREEN = {0, 255, 0};
        public static int[] BLUE = {0, 0, 255};
        public static int[] CYAN = {0, 255, 255};
        public static int[] WHITE = {0, 0, 0};
        public static int[] PINK = {255, 105, 18};
        public static int[] GOLD = {212, 175, 55};
        public static int[] PURPLE = {128, 0, 128};
        public static int[] RUFOUS = {168, 28, 7};

        public static int[] ORANGE_RED = {255, 69, 0};
        public static int[] MAROON = {128, 0, 0};
    }
}
