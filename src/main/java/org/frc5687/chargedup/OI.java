/* Team 5687 (C)2020-2021 */
package org.frc5687.chargedup;

import static org.frc5687.chargedup.util.Helpers.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5687.chargedup.commands.SemiAuto.SemiAutoGroundPickup;
import org.frc5687.chargedup.commands.SemiAuto.SemiAutoPickup;
import org.frc5687.chargedup.commands.SemiAuto.SemiAutoPlaceHigh;
import org.frc5687.chargedup.commands.SemiAuto.SemiAutoPlaceMiddle;
import org.frc5687.chargedup.commands.Tap;
import org.frc5687.chargedup.subsystems.*;
import org.frc5687.chargedup.util.OutliersProxy;
import org.frc5687.lib.logging.RioLogger;
import org.frc5687.lib.oi.AxisButton;
import org.frc5687.lib.oi.Gamepad;
import org.frc5687.lib.oi.Gamepad.Axes;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    protected Trigger _driverLeftTrigger;
    protected Trigger _driverRightTrigger;

    public OI() {
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _driverLeftTrigger =
                new Trigger(
                        new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05)::get);
        _driverRightTrigger =
                new Trigger(
                        new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05)::get);
    }

    public void initializeButtons(
            DriveTrain drivetrain, EndEffector endEffector, Arm arm, Elevator elevator) {
        _operatorGamepad
                .getBackButton()
                .onTrue(Commands.runOnce(endEffector::setConeMode, endEffector));
        _operatorGamepad
                .getStartButton()
                .onTrue(Commands.runOnce(endEffector::setCubeMode, endEffector));
        _operatorGamepad.getAButton().onTrue(new SemiAutoPickup(arm, endEffector, elevator, this));
        _operatorGamepad.getBButton().onTrue(new SemiAutoPlaceMiddle(arm, endEffector, elevator, this));
        _operatorGamepad
                .getXButton()
                .onTrue(new SemiAutoGroundPickup(arm, endEffector, elevator, this));
        _operatorGamepad.getYButton().onTrue(new SemiAutoPlaceHigh(arm, endEffector, elevator, this));
        _driverLeftTrigger.onTrue(new Tap(drivetrain, false));
        _driverRightTrigger.onTrue(new Tap(drivetrain, true));

        //        _driverGamepad
        //                .getYButton()
        //                .onTrue(
        //                        new SequentialCommandGroup(
        //                                new AutoPlaceHighCube(arm, endEffector, elevator),
        //                                new DriveUntilLevel(drivetrain)));
        //        _driverGamepad
        //                .getXButton()
        //                .onTrue(new DriveToPose(drivetrain, new Pose2d(14.6, 2.1,
        // Rotation2d.fromDegrees(180.0))));
        //        _driverGamepad
        //                .getBButton()
        //                .onTrue(new DriveToPose(drivetrain, new Pose2d(14.6, 4.4,
        // Rotation2d.fromDegrees(180.0))));
        //        _operatorGamepad.getBButton().onTrue(new AutoSetWristAngle(
        //                endEffector, Constants.EndEffector.WRIST_MAX_ANGLE));
        //        _operatorGamepad.getAButton().onTrue(new AutoSetWristAngle(
        //                endEffector, Constants.EndEffector.WRIST_MIN_ANGLE
        //        ));
    }

    // TODO: Need to update the gamepad class for 2023 new stuff
    public boolean autoAim() {
        return _driverGamepad.getRightStickButton().getAsBoolean();
    }

    public boolean releaseRoller() {
        return _driverGamepad.getAButton().getAsBoolean();
    }

    public boolean manualGrip() {
        return _operatorGamepad.getLeftBumper().getAsBoolean();
    }

    public boolean setHeadingNorth() {
        return _driverGamepad.getYButton().getAsBoolean();
    }

    public boolean setHeadingEast() {
        return _driverGamepad.getBButton().getAsBoolean();
    }

    //    public boolean setHeadingSouth() {
    //        return _driverGamepad.getAButton().getAsBoolean();
    //    }

    public boolean setHeadingWest() {
        return _driverGamepad.getXButton().getAsBoolean();
    }

    public boolean zeroIMU() {
        return _driverGamepad.getStartButton().getAsBoolean();
    }

    public double getDriveY() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.TRANSLATION_DEADBAND);
        return speed;
    }

    public double getDriveX() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.TRANSLATION_DEADBAND);
        return speed;
    }

    public double getRotationX() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.ROTATION_DEADBAND);
        return speed;
    }

    public double getArmY() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.ROTATION_DEADBAND);
        return speed / 5; // for testing
        //  return 0;
    }

    public double getExtArmY() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.ROTATION_DEADBAND);
        return speed;
    }

    public boolean setCORRight(){
        if (_driverGamepad.getRawAxis(Axes.RIGHT_TRIGGER) > 0.0){
            return true;
        } else {
            return false;
        }
    }

    public boolean setCORLeft(){
        if (_driverGamepad.getRawAxis(Axes.LEFT_TRIGGER) > 0.0){
        return true;
        } else {
            return false;
        }
    }

    // public double getCOR(){
    //     double cor = 0.0;
    //     if (Gamepad.Axes.RIGHT_TRIGGER.getNumber() > 0.0){
    //     cor = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber());
    //     } else if (Gamepad.Axes.LEFT_TRIGGER.getNumber() > 0.0){
    //     cor = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber());
    //     }
    //     return cor;
    // }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {
        // metric("Raw x", xIn);
        // metric("Raw y", yIn);
    }

    public double getRollerSpeed() {
        //  double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.A.getNumber());
        //  speed = applyDeadband(speed, ROTATION_DEADBAND);
        //  return speed;
        return 0;
    }

    public boolean getIntakeIn() {
        return _operatorGamepad.getAButton().getAsBoolean();
    }

    public boolean getIntakeOut() {
        return _operatorGamepad.getBButton().getAsBoolean();
    }

    public boolean getSlowMode() {
        return _driverGamepad.getLeftStickButton().getAsBoolean();
    }

    public double getWristSpeed() {
        //        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        //        speed = applyDeadband(speed, ROTATION_DEADBAND);
        //        return speed; //for testing
        return 0;
    }
}
