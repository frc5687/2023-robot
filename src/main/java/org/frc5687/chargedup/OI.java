/* Team 5687 (C)2020-2021 */
package org.frc5687.chargedup;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.frc5687.lib.oi.Gamepad;
import org.frc5687.chargedup.subsystems.*;

import org.frc5687.chargedup.util.OutliersProxy;

import static org.frc5687.chargedup.Constants.DriveTrain.*;
import static org.frc5687.chargedup.util.Helpers.*;

import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.commands.Arm.AutoSetArmSetpoint;
import org.frc5687.chargedup.commands.Arm.DriveUntilHall;
// import org.frc5687.chargedup.commands.EndEffector.AutoSetGripperAngle;
import org.frc5687.chargedup.commands.EndEffector.AutoSetWristAngle;
import org.frc5687.chargedup.commands.Elevator.AutoExtendElevator;

import java.util.concurrent.ConcurrentMap;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;


    public OI() {
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
    }

    public void initializeButtons(EndEffector endEffector, Arm arm, Elevator elevator) {
//        _operatorGamepad.getAButton().onTrue(new AutoSetSuperStructurePosition(
//            elevator, endEffector, arm, 0.0, Constants.EndEffector.WRIST_PICKUP_ANGLE,
//            Constants.EndEffector.GRIPPER_OPEN_ANGLE, 1.51
//        ));
//        _operatorGamepad.getBButton().onTrue(new AutoSetSuperStructurePosition(
//            elevator, endEffector, arm, .4, Constants.EndEffector.WRIST_MIN_ANGLE,
//            Constants.EndEffector.GRIPPER_CLOSED_ANGLE, 3.4
//        ));
        _operatorGamepad.getBButton().onTrue(
                new AutoExtendElevator(elevator, .55)
        );
        _operatorGamepad.getAButton().onTrue(
                new AutoExtendElevator(elevator, .2)
        );
        // _operatorGamepad.getAButton().whenPressed(new AutoSetArmSetpoint(arm, 1.51));
        // _operatorGamepad.getBButton().whenPressed(new AutoSetArmSetpoint(arm, 3.4));
        // _operatorGamepad.getXButton().onTrue(new AutoSetRoller(endEffector, Constants.EndEffector.GRIPPER_OPEN_ANGLE));
        // _operatorGamepad.getYButton().onTrue(new AutoSetGripperAngle(endEffector, Constants.EndEffector.GRIPPER_CLOSED_ANGLE));
        // _operatorGamepad.getRightBumper().onTrue(new AutoSetGripperAngle(endEffector, Constants.EndEffector.GRIPPER_CUBE_ANGLE));

    }

    // TODO: Need to update the gamepad class for 2023 new stuff
    public boolean setHeadingNorth() {
        return _driverGamepad.getYButton().getAsBoolean();
    }

    public boolean setHeadingEast() {
        return _driverGamepad.getBButton().getAsBoolean();
    }

    public boolean setHeadingSouth() {
        return _driverGamepad.getAButton().getAsBoolean();
    }

    public boolean setHeadingWest() {
        return _driverGamepad.getXButton().getAsBoolean();
    }
    public boolean zeroIMU() {
        return _driverGamepad.getStartButton().getAsBoolean();
    }

    public double getDriveY() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, TRANSLATION_DEADBAND);
        return speed; 
    }

    public double getDriveX() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, TRANSLATION_DEADBAND);
        return speed;
    }

    public double getRotationX() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, ROTATION_DEADBAND);
        return speed;
    }

    public double getArmY() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, ROTATION_DEADBAND);
        return speed/5; //for testing
    }
    public double getExtArmY(){
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber());
        speed = applyDeadband(speed, ROTATION_DEADBAND);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {
        // metric("Raw x", xIn);
        // metric("Raw y", yIn);
    }
    public double getGripperSpeed() {
        // double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        // speed = applyDeadband(speed, ROTATION_DEADBAND);
        // return speed;
        return 0;
    }

    public double getWristSpeed() {
        // double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        // speed = applyDeadband(speed, ROTATION_DEADBAND);
        // return speed;
        return 0;
    }
}
