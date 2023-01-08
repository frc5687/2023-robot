/* Team 5687 (C)2020-2021 */
package org.frc5687.swerve;

import static org.frc5687.swerve.Constants.DriveTrain.*;
import static org.frc5687.swerve.util.Helpers.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.swerve.subsystems.DriveTrain;
import org.frc5687.swerve.util.AxisButton;
import org.frc5687.swerve.util.Gamepad;
import org.frc5687.swerve.util.OutliersProxy;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Joystick _leftJoystick;
    protected Joystick _rightJoystick;

    protected Button _driverRightStickButton;

    private JoystickButton _trigger;
    private JoystickButton _thumbButton;
    private JoystickButton _shootButton;
    private JoystickButton _resetYawButton;

    private Button _driverAButton;
    private Button _driverBButton;
    private Button _driverXButton;
    private Button _driverYButton;
    private Button _driverRightTrigger;

    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _driverGamepad = new Gamepad(0);

        _leftJoystick = new Joystick(1);
        _rightJoystick = new Joystick(2);

        _driverRightStickButton =
                new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _trigger = new JoystickButton(_rightJoystick, 1);
        _thumbButton = new JoystickButton(_rightJoystick, 2);
        _shootButton = new JoystickButton(_leftJoystick, 1);
        _resetYawButton = new JoystickButton(_rightJoystick, 4);

        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());
        _driverRightTrigger =
                new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.2);
    }

    public void initializeButtons(DriveTrain driveTrain) {}

    public double getDriveY() {
        //        yIn = getSpeedFromAxis(_leftJoystick, _leftJoystick.getYChannel());
        yIn = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        yIn = applyDeadband(yIn, DEADBAND);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        //        xIn = -getSpeedFromAxis(_leftJoystick, _leftJoystick.getXChannel());
        xIn = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        xIn = applyDeadband(xIn, DEADBAND);

        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0;
        return xOut;
    }

    public double getRotationX() {
        double speed = getSpeedFromAxis(_rightJoystick, _rightJoystick.getZChannel());
        speed = applyDeadband(speed, 0.2);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}
}
