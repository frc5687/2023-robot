package org.frc5687.chargedup.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.chargedup.commands.Arm.AutoSetArmSetpoint;
import org.frc5687.chargedup.commands.Elevator.AutoExtendElevator;
import org.frc5687.chargedup.commands.EndEffector.SetEndEffectorPosition;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.util.SuperStructureSetpoints.Setpoint;

public class AutoExtendSuperStructurePosition extends SequentialCommandGroup {
    public AutoExtendSuperStructurePosition(
            Elevator elevator, EndEffector endEffector, Arm arm, Setpoint setpoint) {
        addCommands(
            new AutoSetArmSetpoint(arm, setpoint.armAngle),
            new ParallelCommandGroup(
                new SetEndEffectorPosition(endEffector, setpoint.wristAngle, setpoint.gripperSpeed),
                new AutoExtendElevator(elevator, setpoint.elevatorPosition)
            )
        );
    }
}
