package org.frc5687.chargedup.commands.Auto;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.util.SuperStructureSetpoints.Setpoint;

public class AutoGroundPickupCone extends SequentialCommandGroup {
    public AutoGroundPickupCone(Elevator elevator, Arm arm, EndEffector endEffector) {
        Setpoint groundSetpoint = coneGroundPickupSetpoint;
        Setpoint idleSetpoint = idleConeSetpoint;

        addCommands(
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, groundSetpoint),
                new AutoSetRollerSpeed(endEffector, Constants.EndEffector.GRIPPER_IN_SPEED, true),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, idleSetpoint));
    }
}
