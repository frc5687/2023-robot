/* Team 5687 (C)2020-2021 */
package org.frc5687.chargedup;

import static org.frc5687.chargedup.util.Helpers.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5687.chargedup.commands.Auto.DriveToPose;
import org.frc5687.chargedup.commands.CubeShooter.AutoIntake;
import org.frc5687.chargedup.commands.CubeShooter.AutoRotateWrist;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.commands.Auto.HoverToPose;
import org.frc5687.chargedup.commands.SemiAuto.SemiAutoGroundPickup;
import org.frc5687.chargedup.commands.SemiAuto.SemiAutoPickup;
import org.frc5687.chargedup.commands.SemiAuto.SemiAutoPlaceHigh;
import org.frc5687.chargedup.commands.SemiAuto.SemiAutoPlaceMiddle;
import org.frc5687.chargedup.commands.SetHoverGoal;
import org.frc5687.chargedup.commands.SnapTo;
import org.frc5687.chargedup.commands.Tap;
import org.frc5687.chargedup.subsystems.*;
import org.frc5687.chargedup.util.OutliersProxy;
import org.frc5687.lib.oi.AxisButton;
import org.frc5687.lib.oi.Gamepad;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    protected Gamepad _buttonpad;
    protected Trigger _driverLeftTrigger;
    protected Trigger _driverRightTrigger;
    protected Trigger _buttonLeftTrigger;
    protected Trigger _buttonRightTrigger;

    public OI() {
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _buttonpad = new Gamepad(2);
        _driverLeftTrigger =
                new Trigger(
                        new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05)::get);
        _driverRightTrigger =
                    new Trigger(
                            new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05)::get);
        _buttonLeftTrigger  =
                new Trigger(
                        new AxisButton(_buttonpad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05)::get);
        _buttonRightTrigger =
                new Trigger(
                        new AxisButton(_buttonpad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05)::get);
    }

    public void initializeButtons(
            DriveTrain drivetrain, EndEffector endEffector, Arm arm, Elevator elevator, CubeShooter cubeShooter, Lights lights) {
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
//        _driverLeftTrigger.onTrue(new Tap(drivetrain, false));
        //        _driverRightTrigger.onTrue(new Tap(drivetrain, true));
        _driverRightTrigger.onTrue(new Shoot(cubeShooter));
        _driverLeftTrigger.onTrue(new AutoIntake(cubeShooter, this));

        _driverGamepad
                .getYButton()
                .onTrue(new SnapTo(drivetrain, new Rotation2d(Units.degreesToRadians(0))));
        _driverGamepad.getBButton().whileTrue(new HoverToPose(drivetrain, lights, this));
        // _buttonpad.getLeftBumper().onTrue(new DriveToPose(drivetrain, Constants.Auto.RED_FIRST_GOAL, this));
        // _buttonpad.getXButton().onTrue(new DriveToPose(drivetrain, Constants.Auto.RED_SECOND_GOAL, this));
        // _buttonpad.getYButton().onTrue(new DriveToPose(drivetrain, Constants.Auto.RED_THIRD_GOAL, this));
        // _buttonpad.getRightBumper().onTrue(new DriveToPose(drivetrain, Constants.Auto.RED_FOURTH_GOAL, this));
        // _buttonLeftTrigger.onTrue(new DriveToPose(drivetrain, Constants.Auto.RED_FIFTH_GOAL, this));
        // _buttonpad.getAButton().onTrue(new DriveToPose(drivetrain, Constants.Auto.RED_SIXTH_GOAL, this));
        // _buttonpad.getBButton().onTrue(new DriveToPose(drivetrain, Constants.Auto.RED_SEVENTH_GOAL, this));
        // _buttonRightTrigger.onTrue(new DriveToPose(drivetrain, Constants.Auto.RED_EIGHTH_GOAL, this));
        // _buttonpad.getRightStickButton().onTrue(new DriveToPose(drivetrain, Constants.Auto.RED_NINTH_GOAL, this));
        _buttonpad.getLeftBumper().onTrue(new SetHoverGoal(drivetrain, Constants.Auto.RED_FIRST_GOAL));
        _buttonpad.getXButton().onTrue(new SetHoverGoal(drivetrain, Constants.Auto.RED_SECOND_GOAL));
        _buttonpad.getYButton().onTrue(new SetHoverGoal(drivetrain, Constants.Auto.RED_THIRD_GOAL));
        _buttonpad.getRightBumper().onTrue(new SetHoverGoal(drivetrain, Constants.Auto.RED_FOURTH_GOAL));
        _buttonLeftTrigger.onTrue(new SetHoverGoal(drivetrain, Constants.Auto.RED_FIFTH_GOAL));
        _buttonpad.getAButton().onTrue(new SetHoverGoal(drivetrain, Constants.Auto.RED_SIXTH_GOAL));
        _buttonpad.getBButton().onTrue(new SetHoverGoal(drivetrain, Constants.Auto.RED_SEVENTH_GOAL));
        _buttonRightTrigger.onTrue(new SetHoverGoal(drivetrain, Constants.Auto.RED_EIGHTH_GOAL));
        _buttonpad.getRightStickButton().onTrue(new SetHoverGoal(drivetrain, Constants.Auto.RED_NINTH_GOAL));
        
    }

    // TODO: Need to update the gamepad class for 2023 new stuff
    public boolean autoAim() {
        return _driverGamepad.getAButton().getAsBoolean();
        // return false;
    }

    public boolean releaseRoller() {
        return _driverGamepad.getRightBumper().getAsBoolean();
    }

    public boolean getSlowMode() {
        return _driverGamepad.getLeftBumper().getAsBoolean();
    }

    public boolean zeroIMU() {
        return _driverGamepad.getStartButton().getAsBoolean();
    }

    public boolean manualGrip() {
        return _operatorGamepad.getLeftBumper().getAsBoolean();
    }

    public boolean getIntakeIn() {
        return _operatorGamepad.getAButton().getAsBoolean();
    }

    public boolean getIntakeOut() {
        return _operatorGamepad.getBButton().getAsBoolean();
    }

    // public boolean getTapRight(){
    //     return _driverRightTrigger.getAsBoolean();
    // }

    // public boolean getTapLeft(){
    //     return _driverLeftTrigger.getAsBoolean();
    // }
    public boolean getCubeIntake() {
        return _driverLeftTrigger.getAsBoolean();
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
//        return 0;
    }

    public double getCSWrist() {
//        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber());
//        speed = applyDeadband(speed, Constants.DriveTrain.ROTATION_DEADBAND);
//        return speed;
        return 0;
    }

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

    public double getWristSpeed() {
        //        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        //        speed = applyDeadband(speed, ROTATION_DEADBAND);
        //        return speed; //for testing
        return 0;
    }
}
