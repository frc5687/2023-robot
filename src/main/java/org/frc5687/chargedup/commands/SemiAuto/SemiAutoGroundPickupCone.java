package org.frc5687.chargedup.commands.SemiAuto;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.EndEffector.WaitForPlace;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SemiAutoGroundPickupCone extends SequentialCommandGroup {
    public SemiAutoGroundPickupCone(Arm arm, EndEffector endEffector, Elevator elevator, OI oi) {
        Setpoint setpoint = coneGroundPickupSetpoint;
        addCommands(
                // new DriveUntilInHall(elevator),
                // new AutoExtendElevator(elevator, 0.01),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, setpoint),
                new ParallelDeadlineGroup(
                        new WaitForPlace(endEffector, oi, false), new HoldArm(arm, setpoint.armAngle)),

                //                new CloseConeGripper(endEffector),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, idleConeSetpoint));
    }
}
