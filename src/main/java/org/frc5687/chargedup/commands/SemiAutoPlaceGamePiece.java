package org.frc5687.chargedup.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.EndEffector.AutoSetGripperAngle;
import org.frc5687.chargedup.commands.EndEffector.WaitForManualGripper;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;

public class SemiAutoPlaceGamePiece extends SequentialCommandGroup {
    public SemiAutoPlaceGamePiece(
            Arm arm,
            EndEffector endEffector,
            Elevator elevator,
            OI oi,
            Lights lights
    ) {
        addCommands(
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0.5, Constants.EndEffector.WRIST_MIN_ANGLE,
                        Constants.EndEffector.GRIPPER_CLOSED_ANGLE, Constants.Arm.PLACE_ARM_ANGLE
                ),
                new WaitForManualGripper(oi, lights),
                new AutoSetGripperAngle(endEffector, Constants.EndEffector.GRIPPER_OPEN_ANGLE),
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, 0.2, Constants.EndEffector.WRIST_MID_ANGLE,
                        Constants.EndEffector.GRIPPER_OPEN_ANGLE, Constants.Arm.VERTICAL_ARM_ANGLE
                )
        );
    }
}
