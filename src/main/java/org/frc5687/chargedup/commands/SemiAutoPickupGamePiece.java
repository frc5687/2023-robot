package org.frc5687.chargedup.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.EndEffector.AutoSetGripperAngle;
import org.frc5687.chargedup.commands.EndEffector.WaitForManualGripper;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SemiAutoPickupGamePiece extends SequentialCommandGroup {
    public SemiAutoPickupGamePiece(
            Arm arm,
            EndEffector endEffector,
            Elevator elevator,
            OI oi,
    ) {
        addCommands(
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0.0, Constants.EndEffector.WRIST_PICKUP_ANGLE,
                        Constants.EndEffector.GRIPPER_OPEN_ANGLE, 1.51
                ),
                new WaitForManualGripper(oi),
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0.2, Constants.EndEffector.WRIST_MID_ANGLE,
                        Constants.EndEffector.GRIPPER_CLOSED_ANGLE, Constants.Arm.VERTICAL_ARM_ANGLE
                )
        );
    }
}
