package org.frc5687.chargedup.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.commands.EndEffector.WaitForManualGripper;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SemiAutoPlaceMiddleGamePiece extends SequentialCommandGroup {
    public SemiAutoPlaceMiddleGamePiece(
            Arm arm,
            EndEffector endEffector,
            Elevator elevator,
            OI oi
    ) {
        addCommands(
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0., Constants.EndEffector.WRIST_MIN_ANGLE,
                        /*endEffector.getConeMode() ? */ 0.0 /* : Constants.EndEffector.GRIPPER_CUBE_ANGLE*/, Constants.Arm.PLACE_ARM_ANGLE
                ),
                new ParallelDeadlineGroup(
                        new WaitForManualGripper(endEffector, oi),
                        new HoldArm(arm, Constants.Arm.PLACE_ARM_ANGLE)),

                new AutoSetRollerSpeed(endEffector, Constants.EndEffector.GRIPPER_OUT_SPEED, true),
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0.1, Constants.EndEffector.WRIST_MID_ANGLE,
                        0.0, Constants.Arm.VERTICAL_ARM_ANGLE
                )
        );
    }
}
