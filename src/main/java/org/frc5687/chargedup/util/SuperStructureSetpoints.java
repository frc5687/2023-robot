package org.frc5687.chargedup.util;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.subsystems.Elevator;

public class SuperStructureSetpoints {
    public static class Setpoint {
        public double elevatorPosition = 0.0;
        public double wristAngle = Constants.EndEffector.WRIST_MID_ANGLE;
        public double gripperSpeed = 0.0;
        public double armAngle = Constants.Arm.VERTICAL_ARM_ANGLE;
    }

    public static final Setpoint highConePlaceSetpoint = new Setpoint();
    static {
        highConePlaceSetpoint.elevatorPosition = 0.55;
        highConePlaceSetpoint.wristAngle = Constants.EndEffector.WRIST_MIN_ANGLE;
        highConePlaceSetpoint.gripperSpeed = 0.0;
        highConePlaceSetpoint.armAngle = Constants.Arm.PLACE_ARM_ANGLE;
    }

    public static final Setpoint middleConePlaceSetpoint = new Setpoint();
    static {
        middleConePlaceSetpoint.elevatorPosition = 0.0;
        middleConePlaceSetpoint.wristAngle = Constants.EndEffector.WRIST_MIN_ANGLE;
        middleConePlaceSetpoint.gripperSpeed = 0.0;
        middleConePlaceSetpoint.armAngle = Constants.Arm.PLACE_ARM_ANGLE;
    }

    public final static Setpoint highCubePlaceSetpoint = new Setpoint();
    static {
        highConePlaceSetpoint.elevatorPosition = 0.55;
        highConePlaceSetpoint.wristAngle = Constants.EndEffector.WRIST_MIN_ANGLE;
        highConePlaceSetpoint.gripperSpeed = 0.0;
        highConePlaceSetpoint.armAngle = Constants.Arm.PLACE_ARM_ANGLE;
    }

    public static final Setpoint middleCubePlaceSetpoint = new Setpoint();
    static {
        middleConePlaceSetpoint.elevatorPosition = 0.0;
        middleConePlaceSetpoint.wristAngle = Constants.EndEffector.WRIST_MIN_ANGLE;
        middleConePlaceSetpoint.gripperSpeed = 0.0;
        middleConePlaceSetpoint.armAngle = Constants.Arm.PLACE_ARM_ANGLE;
    }

    public static final Setpoint conePickupSetpoint = new Setpoint();
    static {
        conePickupSetpoint.elevatorPosition = 0.385;
        conePickupSetpoint.wristAngle = Constants.EndEffector.WRIST_PICKUP_ANGLE;
        conePickupSetpoint.gripperSpeed = Constants.EndEffector.GRIPPER_IN_SPEED;
        conePickupSetpoint.armAngle = 1.72;
    }

    public static final Setpoint cubePickupSetpoint = new Setpoint();
    static {
        cubePickupSetpoint.elevatorPosition = 0.345;
        cubePickupSetpoint.wristAngle = Constants.EndEffector.WRIST_PICKUP_ANGLE;
        cubePickupSetpoint.gripperSpeed = -Constants.EndEffector.GRIPPER_IN_SPEED;
        cubePickupSetpoint.armAngle = 1.72;
    }

    public static final Setpoint idleSetpoint = new Setpoint();
    static {
        idleSetpoint.elevatorPosition = 0.0;
        idleSetpoint.wristAngle = Constants.EndEffector.WRIST_MID_ANGLE;
        idleSetpoint.gripperSpeed = 0.0;
        idleSetpoint.armAngle = Constants.Arm.VERTICAL_ARM_ANGLE;
    }

}
