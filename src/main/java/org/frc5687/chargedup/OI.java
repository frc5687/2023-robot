/* Team 5687 (C)2020-2021 */
package org.frc5687.chargedup;

import static org.frc5687.chargedup.util.Helpers.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5687.chargedup.commands.Auto.HoverToPose;
import org.frc5687.chargedup.commands.CubeShooter.AutoIntake;
import org.frc5687.chargedup.commands.CubeShooter.AutoShoot;
import org.frc5687.chargedup.commands.SemiAuto.*;
import org.frc5687.chargedup.commands.SetRobotGoal;
import org.frc5687.chargedup.commands.SnapTo;
import org.frc5687.chargedup.commands.Tap;
import org.frc5687.chargedup.commands.ZeroSuperStructure;
import org.frc5687.chargedup.subsystems.*;
import org.frc5687.chargedup.subsystems.EndEffector.IntakeState;
import org.frc5687.chargedup.util.CustomController;
import org.frc5687.chargedup.util.Nodes;
import org.frc5687.chargedup.util.OutliersProxy;
import org.frc5687.lib.oi.AxisButton;
import org.frc5687.lib.oi.Gamepad;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected CommandJoystick _operatorJoystick;
    protected Gamepad _buttonpad;

    protected CustomController _customController;
    protected Trigger _driverLeftTrigger;
    protected Trigger _driverRightTrigger;
    protected Trigger _buttonLeftTrigger;
    protected Trigger _buttonRightTrigger;
    protected Trigger _povButtonRight;
    protected Trigger _povButtonLeft;

    public OI() {
        _driverGamepad = new Gamepad(0);
        _operatorJoystick = new CommandJoystick(1);
        _buttonpad = new Gamepad(2);
        _customController = new CustomController();
        _povButtonLeft = new Trigger(() -> _driverGamepad.getPOV() == 270);
        _povButtonRight = new Trigger(() -> _driverGamepad.getPOV() == 90);
        _driverLeftTrigger =
                new Trigger(
                        new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05)::get);
        _driverRightTrigger =
                new Trigger(
                        new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05)::get);
        _buttonLeftTrigger =
                new Trigger(new AxisButton(_buttonpad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05)::get);
        _buttonRightTrigger =
                new Trigger(new AxisButton(_buttonpad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05)::get);
    }

    public void initializeButtons(
            DriveTrain drivetrain,
            EndEffector endEffector,
            Arm arm,
            Elevator elevator,
            CubeShooter cubeShooter,
            Lights lights) {
        _customController
                .getChangeModeButton()
                .toggleOnTrue(Commands.runOnce(endEffector::setConeState, endEffector));
        _customController
                .getChangeModeButton()
                .toggleOnFalse(Commands.runOnce(endEffector::setCubeState, endEffector));
        _operatorJoystick.button(6).onTrue(Commands.runOnce(endEffector::setConeState, endEffector));
        _operatorJoystick.button(7).onTrue(Commands.runOnce(endEffector::setCubeState, endEffector));

        _operatorJoystick.button(8).and(_operatorJoystick.button(9)).onTrue(new ZeroSuperStructure(elevator, arm, endEffector));

        _customController
                .getIntakeButton()
                .onTrue(new SemiAutoPickup(arm, endEffector, elevator, this));

        _operatorJoystick.button(2).onTrue(new SemiAutoPickup(arm, endEffector, elevator, this));
        _operatorJoystick.button(4).onTrue(new SemiAutoPlaceMiddle(arm, endEffector, elevator, this));
        _operatorJoystick.button(5).onTrue(new SemiAutoPlaceHigh(arm, endEffector, elevator, this));

        _povButtonLeft.onTrue(new Tap(drivetrain, false));
        _povButtonRight.onTrue(new Tap(drivetrain, true));

        //        _driverGamepad.getXButton().onTrue(new
        // DriveToPose(Constants.Auto.FieldPoses.RED_TARGET_FOUR))
        _driverRightTrigger.onTrue(new AutoShoot(cubeShooter, drivetrain, endEffector, this));
        _driverGamepad
                .getRightBumper()
                .onTrue(
                        new AutoShoot(cubeShooter, drivetrain, endEffector, this)
                                .unless(() -> !cubeShooter.isCubeDetected()));
        _driverLeftTrigger.whileTrue(new AutoIntake(cubeShooter, false));

        _driverGamepad
                .getYButton()
                .onTrue(new SnapTo(drivetrain, new Rotation2d(Units.degreesToRadians(0))));
        _driverGamepad.getAButton().onTrue(new SnapTo(drivetrain, new Rotation2d(Units.degreesToRadians(180))));
        _driverGamepad.getBButton().whileTrue(new HoverToPose(drivetrain, cubeShooter, lights));
        _operatorJoystick
                .button(1)
                .onTrue(new SemiAutoPlace(arm, endEffector, elevator, cubeShooter, drivetrain, this));
        _customController
                .getDeployButton()
                .onTrue(new SemiAutoPlace(arm, endEffector, elevator, cubeShooter, drivetrain, this));
    }

    // TODO: Need to update the gamepad class for 2023 new stuff
    public boolean autoAim() {
        return _driverGamepad.getXButton().getAsBoolean();
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

    public boolean stow() {
        return _operatorJoystick.button(3).getAsBoolean();
        // return false;
    }

    public boolean getIntakeIn() {
        // return _operatorGamepad.getAButton().getAsBoolean();
        return false;
    }

    public boolean getIntakeOut() {
        // return _operatorGamepad.getBButton().getAsBoolean();
        return false;
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
        // double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        // speed = applyDeadband(speed, Constants.DriveTrain.ROTATION_DEADBAND);
        // return speed / 5; // for testing
        return 0;
    }

    public double getExtArmY() {
        // double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber());
        // speed = applyDeadband(speed, Constants.DriveTrain.ROTATION_DEADBAND);
        // return speed;
        return 0;
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
