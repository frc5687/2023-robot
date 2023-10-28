/* Team 5687 (C)2020-2021 */
package org.frc5687.chargedup;

import static org.frc5687.chargedup.util.Helpers.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5687.chargedup.commands.*;
import org.frc5687.chargedup.commands.Auto.HoverToPose;
import org.frc5687.chargedup.commands.CubeShooter.AutoIntake;
import org.frc5687.chargedup.commands.CubeShooter.AutoShoot;
import org.frc5687.chargedup.commands.EndEffector.EjectStow;
import org.frc5687.chargedup.commands.SemiAuto.*;
import org.frc5687.chargedup.subsystems.*;
import org.frc5687.chargedup.util.CustomController;
import org.frc5687.chargedup.util.Nodes;
import org.frc5687.chargedup.util.OutliersProxy;
import org.frc5687.lib.oi.AxisButton;
import org.frc5687.lib.oi.Gamepad;
import org.frc5687.lib.sensors.ProximitySensor;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected CommandJoystick _operatorJoystick;
    protected Gamepad _buttonpad;

    protected CustomController _customController;
    protected Trigger _driverLeftTrigger;
    protected Trigger _driverRightTrigger;
    protected Trigger _buttonLeftTrigger;
    protected Trigger _buttonRightTrigger;
    protected Trigger _povButtonLeft;
    protected Trigger _povButtonRight;
    protected Trigger _povButtonUp;
    protected Trigger _povButtonDown;
    
    public OI() {
    
        _driverGamepad = new Gamepad(0);
        _operatorJoystick = new CommandJoystick(1);
        _buttonpad = new Gamepad(2);
        _customController = new CustomController();
        _povButtonLeft = new Trigger(() -> _driverGamepad.getPOV() == 270);
        _povButtonRight = new Trigger(() -> _driverGamepad.getPOV() == 90);
        _povButtonUp = new Trigger(() -> _driverGamepad.getPOV() == 0);
        _povButtonDown = new Trigger(() -> _driverGamepad.getPOV() == 180);
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
                .toggleOnTrue(Commands.runOnce(endEffector::setConeState));
        _customController
                .getChangeModeButton()
                .toggleOnFalse(Commands.runOnce(endEffector::setCubeState));
        _operatorJoystick.button(6).onTrue(Commands.runOnce(endEffector::setConeState));
        _operatorJoystick.button(7).onTrue(Commands.runOnce(endEffector::setCubeState));

        _operatorJoystick.button(8).or(_operatorJoystick.button(9)).onTrue(new ZeroSuperStructure(elevator, arm, endEffector));
        _operatorJoystick.button(10).or(_operatorJoystick.button(11)).onTrue(new EjectStow(endEffector, elevator, arm));

        _customController
                .getIntakeButton()
                .onTrue(new SemiAutoPickup(arm, endEffector, elevator, this));

        _operatorJoystick.button(2).onTrue(new SemiAutoPickup(arm, endEffector, elevator, this));
        _operatorJoystick.button(4).onTrue(new SemiAutoPlaceMiddle(arm, endEffector, elevator, this));
        _operatorJoystick.button(5).onTrue(new SemiAutoPlaceHigh(arm, endEffector, elevator, this));

        // _povButtonLeft.onTrue(new Tap(drivetrain, false));
        // _povButtonRight.onTrue(new Tap(drivetrain, true));

        _povButtonLeft.whileTrue(new DriveWithSpeeds(drivetrain, 0, 1));
        _povButtonRight.whileTrue(new DriveWithSpeeds(drivetrain, 0, -1));
        _povButtonUp.whileTrue(new DriveWithSpeeds(drivetrain, 1, 0));
        _povButtonDown.whileTrue(new DriveWithSpeeds(drivetrain, -1, 0));


        //        _driverGamepad.getXButton().onTrue(new
        // DriveToPose(Constants.Auto.FieldPoses.RED_TARGET_FOUR))
        _driverRightTrigger.onTrue(new AutoShoot(cubeShooter, drivetrain, endEffector, this));
        _driverGamepad
                .getRightBumper()
                .onTrue(
                        new AutoShoot(cubeShooter, drivetrain, endEffector, this)
                                .unless(() -> !cubeShooter.isCubeDetected()));
        _driverLeftTrigger.whileTrue(new AutoIntake(cubeShooter, false, this));

        _driverGamepad
                .getYButton()
                .onTrue(new SnapTo(drivetrain, new Rotation2d(Units.degreesToRadians(0))));
//        _driverGamepad
//                .getYButton()
//                .onTrue(new CharacterizeModule(drivetrain));
        // _driverGamepad.getAButton().onTrue(new SnapTo(drivetrain, new Rotation2d(Units.degreesToRadians(180))));
        // _driverGamepad.getBButton().whileTrue(new HoverToPose(drivetrain, cubeShooter, lights));
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 9; col++) {
                _customController
                        .getButton(row, col)
                        .onTrue(
                                new SetRobotGoal(
                                        drivetrain, endEffector, Nodes.Node.values()[col], Nodes.Level.values()[row]));
            }
        }
        _operatorJoystick
                .button(1)
                .onTrue(new SemiAutoPlace(arm, endEffector, elevator, cubeShooter, drivetrain, this));
        _customController
                .getDeployButton()
                .onTrue(new SemiAutoPlace(arm, endEffector, elevator, cubeShooter, drivetrain, this));
    
    }

   
    public boolean autoAim() {
        return _driverGamepad.getXButton().getAsBoolean();
        // return false;
    }

    public boolean shiftUp(){
        return _driverGamepad.getAButton().getAsBoolean();
    }

    public boolean shiftDown(){
        return _driverGamepad.getBButton().getAsBoolean();
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
    public void RumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }
    public void stopRumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 0);

    }
}
