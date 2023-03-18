package org.frc5687.chargedup.util;

import edu.wpi.first.math.util.Units;
import org.frc5687.chargedup.Constants;

public class SuperStructureSetpoints {
    public static class Setpoint {
        public double elevatorPosition = 0.0;
        public double wristAngle = Constants.EndEffector.WRIST_MID_ANGLE;
        public double gripperSpeed = 0.0;
        public double armAngle = Constants.Arm.VERTICAL_ARM_ANGLE;
        public double placeSpeed = 0.0;
    }

    public static final Setpoint highConePlaceSetpoint = new Setpoint();

    static {
        highConePlaceSetpoint.elevatorPosition = 0.55;
        highConePlaceSetpoint.wristAngle = Constants.EndEffector.WRIST_MIN_ANGLE;
        highConePlaceSetpoint.gripperSpeed = Constants.EndEffector.ROLLER_CONE_IDLE_SPEED;
        highConePlaceSetpoint.armAngle = Constants.Arm.PLACE_ARM_ANGLE;
        highConePlaceSetpoint.placeSpeed = Constants.EndEffector.PLACE_CONE_ROLLER_SPEED;
    }

    public static final Setpoint middleConePlaceSetpoint = new Setpoint();

    static {
        middleConePlaceSetpoint.elevatorPosition = 0.0;
        middleConePlaceSetpoint.wristAngle = Constants.EndEffector.WRIST_MIN_ANGLE;
        middleConePlaceSetpoint.gripperSpeed = Constants.EndEffector.ROLLER_CONE_IDLE_SPEED;
        middleConePlaceSetpoint.armAngle = Constants.Arm.PLACE_ARM_ANGLE;
        middleConePlaceSetpoint.placeSpeed = Constants.EndEffector.PLACE_CONE_ROLLER_SPEED;
    }

    public static final Setpoint highCubePlaceSetpoint = new Setpoint();

    static {
        highCubePlaceSetpoint.elevatorPosition = 0.55;
        highCubePlaceSetpoint.wristAngle = Constants.EndEffector.WRIST_MIN_ANGLE;
        highCubePlaceSetpoint.gripperSpeed = Constants.EndEffector.ROLLER_CUBE_IDLE_SPEED;
        highCubePlaceSetpoint.armAngle = Constants.Arm.PLACE_ARM_ANGLE - 0.3;
        highCubePlaceSetpoint.placeSpeed = Constants.EndEffector.PLACE_CUBE_ROLLER_SPEED;
    }

    public static final Setpoint middleCubePlaceSetpoint = new Setpoint();

    static {
        middleCubePlaceSetpoint.elevatorPosition = 0.0;
        middleCubePlaceSetpoint.wristAngle = Constants.EndEffector.WRIST_MIN_ANGLE;
        middleCubePlaceSetpoint.gripperSpeed = Constants.EndEffector.ROLLER_CUBE_IDLE_SPEED;
        middleCubePlaceSetpoint.armAngle = Constants.Arm.PLACE_ARM_ANGLE - 0.3;
        middleCubePlaceSetpoint.placeSpeed = Constants.EndEffector.PLACE_CUBE_ROLLER_SPEED;
    }

    public static final Setpoint conePickupSetpoint = new Setpoint();

    static {
        conePickupSetpoint.elevatorPosition = 0.297;
        conePickupSetpoint.wristAngle = Constants.EndEffector.WRIST_PICKUP_ANGLE;
        conePickupSetpoint.gripperSpeed = Constants.EndEffector.GRIPPER_IN_SPEED;
        conePickupSetpoint.armAngle = 1.86;
    }

    public static final Setpoint cubePickupSetpoint = new Setpoint();

    static {
        cubePickupSetpoint.elevatorPosition = 0.28;
        cubePickupSetpoint.wristAngle = Units.degreesToRadians(295);
        cubePickupSetpoint.gripperSpeed = -Constants.EndEffector.GRIPPER_IN_SPEED;
        cubePickupSetpoint.armAngle = 1.86;
    }

    public static final Setpoint coneGroundPickupSetpoint = new Setpoint();

    static {
        coneGroundPickupSetpoint.elevatorPosition = 0.19;
        coneGroundPickupSetpoint.wristAngle = Units.degreesToRadians(260);
        coneGroundPickupSetpoint.gripperSpeed = Constants.EndEffector.GRIPPER_IN_SPEED;
        //        coneGroundPickupSetpoint.armAngle = 0.7;
        coneGroundPickupSetpoint.armAngle = 2.85;
    }

    public static final Setpoint cubeGroundPickupSetpoint = new Setpoint();

    static {
        cubeGroundPickupSetpoint.elevatorPosition = 0.163;
        cubeGroundPickupSetpoint.wristAngle = Units.degreesToRadians(257);
        cubeGroundPickupSetpoint.gripperSpeed = -Constants.EndEffector.GRIPPER_IN_SPEED;
        //        cubeGroundPickupSetpoint.armAngle = 0.667;
        cubeGroundPickupSetpoint.armAngle = 3.0;
    }

    public static final Setpoint idleConeSetpoint = new Setpoint();

    static {
        idleConeSetpoint.elevatorPosition = 0.02;
        idleConeSetpoint.wristAngle = Constants.EndEffector.WRIST_MID_ANGLE;
        idleConeSetpoint.gripperSpeed = Constants.EndEffector.ROLLER_CONE_IDLE_SPEED;
        idleConeSetpoint.armAngle = Constants.Arm.VERTICAL_ARM_ANGLE;
    }

    public static final Setpoint idleCubeSetpoint = new Setpoint();

    static {
        idleCubeSetpoint.elevatorPosition = 0.02;
        idleCubeSetpoint.wristAngle = Constants.EndEffector.WRIST_MID_ANGLE;
        idleCubeSetpoint.gripperSpeed = Constants.EndEffector.ROLLER_CUBE_IDLE_SPEED;
        idleCubeSetpoint.armAngle = Constants.Arm.VERTICAL_ARM_ANGLE;
    }
}
