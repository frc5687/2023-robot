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
import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

public class SemiAutoPlaceHighGamePiece extends SequentialCommandGroup {
    public SemiAutoPlaceHighGamePiece(
            Arm arm,
            EndEffector endEffector,
            Elevator elevator,
            OI oi
    ) {
        Setpoint setpoint = endEffector.getConeMode() ? middleConePlaceSetpoint : middleCubePlaceSetpoint;
        addCommands(
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, setpoint
                ),
                new ParallelDeadlineGroup(
                        new WaitForManualGripper(endEffector, oi),
                        new HoldArm(arm, setpoint.armAngle)),

                new AutoSetRollerSpeed(endEffector, setpoint.gripperSpeed, true),
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, idleSetpoint
                )
        );
    }
}
