/* Team 5687 (C)2020-2021 */
package org.frc5687.swerve;

import static org.frc5687.swerve.util.Helpers.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5687.swerve.commands.*;
import org.frc5687.swerve.subsystems.*;
import org.frc5687.swerve.util.OutliersProxy;
import org.frc5687.lib.oi.AxisButton;
import org.frc5687.lib.oi.Gamepad;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected CommandJoystick _operatorJoystick;
    protected Gamepad _buttonpad;

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
    }

    public void initializeButtons(
            DriveTrain drivetrain
            ) {
        // _povButtonLeft.whileTrue(new DriveWithSpeeds(drivetrain, 0, 1));
        // _povButtonRight.whileTrue(new DriveWithSpeeds(drivetrain, 0, -1));
        // _povButtonUp.whileTrue(new DriveWithSpeeds(drivetrain, 1, 0));
        // _povButtonDown.whileTrue(new DriveWithSpeeds(drivetrain, -1, 0));


        _driverGamepad
                .getYButton()
                .onTrue(new SnapTo(drivetrain, new Rotation2d(Units.degreesToRadians(0))));
//        _driverGamepad
//                .getYButton()
//                .onTrue(new CharacterizeModule(drivetrain));
        // _driverGamepad.getAButton().onTrue(new SnapTo(drivetrain, new Rotation2d(Units.degreesToRadians(180))));
    }

    public boolean getSlowMode() {
        return _driverGamepad.getLeftBumper().getAsBoolean();
    }

    public boolean shiftUp(){
        return _driverGamepad.getAButton().getAsBoolean();
    }

    public boolean shiftDown(){
        return _driverGamepad.getBButton().getAsBoolean();
    }
    public boolean shiftOverride(){
        return _driverGamepad.getStartButton().getAsBoolean();
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
    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {
        // metric("Raw x", xIn);
        // metric("Raw y", yIn);
    }
    public void RumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }
    public void stopRumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 0);

    }
}
