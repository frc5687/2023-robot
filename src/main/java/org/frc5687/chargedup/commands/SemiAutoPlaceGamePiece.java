package org.frc5687.chargedup.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.Elevator.DriveUntilInHall;
import org.frc5687.chargedup.commands.EndEffector.AutoSetGripperAngle;
import org.frc5687.chargedup.commands.EndEffector.WaitForManualGripper;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SemiAutoPlaceGamePiece extends SequentialCommandGroup {
    public SemiAutoPlaceGamePiece(
            Arm arm,
            EndEffector endEffector,
            Elevator elevator,
            OI oi
    ) {
        addCommands(
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0.65, Constants.EndEffector.WRIST_MIN_ANGLE,
                        /*endEffector.getConeMode() ? */Constants.EndEffector.GRIPPER_CLOSED_ANGLE /* : Constants.EndEffector.GRIPPER_CUBE_ANGLE*/, Constants.Arm.PLACE_ARM_ANGLE
                ),
                new ParallelDeadlineGroup(
                        new WaitForManualGripper(oi),
                        new HoldArm(arm, Constants.Arm.PLACE_ARM_ANGLE)),
                new AutoSetGripperAngle(endEffector, Constants.EndEffector.GRIPPER_OPEN_ANGLE),
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0.1, Constants.EndEffector.WRIST_MID_ANGLE,
                        Constants.EndEffector.GRIPPER_OPEN_ANGLE, Constants.Arm.VERTICAL_ARM_ANGLE
                )
        );
    }
}
