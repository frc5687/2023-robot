package org.frc5687.chargedup.commands.Arm;

import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.Constants;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ControllerDriveArm extends OutliersCommand {

    private Arm _arm;
    private TrapezoidProfile.State _lastProfiledReference;

    // We need a variable to place the goal of our arm
    private TrapezoidProfile.State goal;
    // In the constructor we need the Arm subsystem and the angle we would like to go to.
    public ControllerDriveArm(Arm arm, double angle) {
        _arm = arm;
        // we should create the goal here which takes the angle and velocity.
        goal = new TrapezoidProfile.State(angle, _arm.getArmVelocityRadPerSec());
        // what should the velocity be in the goal?
        addRequirements(_arm);
    }



    @Override
    public void initialize() {
        // We need to set the previous profiled reference to match the current state of the arm using sensors we have.
        _lastProfiledReference =
        new TrapezoidProfile.State(_arm.getArmAngleRadians(), _arm.getArmVelocityRadPerSec());
    }

    @Override
    public void execute() {
        // Step our TrapezoidalProfile forward 20ms and set it as our next reference
        _lastProfiledReference = (new TrapezoidProfile(_arm.getConstraints(), goal, _lastProfiledReference)).calculate(0.020);
        _arm.setNextReference(_lastProfiledReference);
        // we need to set the arm reference so that our periodic function will give us the correct voltage.

        // we need to send the voltage to the motor, we should call a function here.
        _arm.setArmVoltage(_arm.getNextVoltage());
    }

    @Override
    public boolean isFinished() {
        // This command should finish when the arm has done what we want.
        return Math.abs(goal.position - _arm.getArmAngleRadians()) < Constants.Arm.ANGLE_TOLERANCE;
        //super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
