package org.frc5687.chargedup.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.EndEffector.CloseConeGripper;
import org.frc5687.chargedup.commands.EndEffector.WaitForManualGripper;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SemiAutoPickupGamePiece extends SequentialCommandGroup {
    public SemiAutoPickupGamePiece(
            Arm arm,
            EndEffector endEffector,
            Elevator elevator,
            OI oi
        ) {
        addCommands(
                // new DriveUntilInHall(elevator),
               // new AutoExtendElevator(elevator, 0.01),
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0.37, Constants.EndEffector.WRIST_PICKUP_ANGLE,
                        Constants.EndEffector.GRIPPER_IN_SPEED, 1.67
                ),
                new ParallelDeadlineGroup(new WaitForManualGripper(endEffector, oi), new HoldArm(arm, 1.67)),

//                new CloseConeGripper(endEffector),
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0.1, Constants.EndEffector.WRIST_MID_ANGLE,
                        /*endEffector.getConeMode() ?*/ -0.5 /*: Constants.EndEffector.GRIPPER_CUBE_ANGLE*/, Constants.Arm.VERTICAL_ARM_ANGLE
                )
        );
    }
}
