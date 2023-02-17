package org.frc5687.chargedup.commands.SemiAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.EndEffector.WaitForManualGripper;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

public class SemiAutoPickupCube extends SequentialCommandGroup {
    public SemiAutoPickupCube(
            Arm arm,
            EndEffector endEffector,
            Elevator elevator,
            OI oi
        ) {
        Setpoint setpoint = cubePickupSetpoint;
        addCommands(
                // new DriveUntilInHall(elevator),
               // new AutoExtendElevator(elevator, 0.01),
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, setpoint
                ),
                new ParallelDeadlineGroup(new WaitForManualGripper(endEffector, oi), new HoldArm(arm, setpoint.armAngle)),

//                new CloseConeGripper(endEffector),
                new AutoSetSuperStructurePosition(
                        elevator, endEffector, arm, idleCubeSetpoint
                )
        );
    }
}
