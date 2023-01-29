/* Team 5687 (C)2020-2021 */
package org.frc5687.chargedup;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.frc5687.lib.oi.Gamepad;
import org.frc5687.chargedup.subsystems.*;

import org.frc5687.chargedup.util.OutliersProxy;

import static org.frc5687.chargedup.Constants.DriveTrain.*;
import static org.frc5687.chargedup.util.Helpers.*;

import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.commands.AutoExtendArm;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    

    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
    }

    public void initializeButtons(DriveTrain driveTrain, Trajectory trajectory, ExtendingArm extendingArm) {
       /*  _driverGamepad.getAButton().whenPressed(new DriveTrajectory(driveTrain, trajectory));

        _driverGamepad.getYButton().whileActiveOnce(new AutoExtendArm(extendingArm, Constants.ExtendingArm.SHORT_ARM_DISTANCE));
        _driverGamepad.getXButton().whileActiveOnce(new AutoExtendArm(extendingArm, Constants.ExtendingArm.MEDIUM_ARM_DISTANCE));
        _driverGamepad.getBButton().whileActiveOnce(new AutoExtendArm(extendingArm, Constants.ExtendingArm.LONG_ARM_DISTANCE));
*/
    }

    // TODO: Need to update the gamepad class for 2023 new stuff
    public boolean autoAim() {
        return _driverGamepad.getXButton().getAsBoolean();
    }

    public double getDriveY() {
        yIn = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        yIn = applyDeadband(yIn, TRANSLATION_DEADBAND);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        xIn = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        xIn = applyDeadband(xIn, TRANSLATION_DEADBAND);

        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0;
        return xOut;
    }

    public double getRotationX() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, ROTATION_DEADBAND);
        return speed;
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
        metric("Raw x", xIn);
        metric("Raw y", yIn);
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
