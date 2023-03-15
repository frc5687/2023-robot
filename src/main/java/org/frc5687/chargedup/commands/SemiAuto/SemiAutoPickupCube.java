package org.frc5687.chargedup.commands.SemiAuto;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.Elevator.AutoExtendElevator;
import org.frc5687.chargedup.commands.EndEffector.AutoSetWristAngle;
import org.frc5687.chargedup.commands.EndEffector.HoldWristAngle;
import org.frc5687.chargedup.commands.EndEffector.WaitForPlace;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SemiAutoPickupCube extends SequentialCommandGroup {
    public SemiAutoPickupCube(Arm arm, EndEffector endEffector, Elevator elevator, OI oi) {
        Setpoint setpoint = cubePickupSetpoint;
        addCommands(
                // new DriveUntilInHall(elevator),
                new AutoExtendElevator(elevator, Constants.ExtendingArm.SHORT_ARM_DISTANCE),
                new AutoSetWristAngle(endEffector, Constants.EndEffector.WRIST_SAFE_ANGLE),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, setpoint),
                new ParallelDeadlineGroup(
                        new WaitForPlace(endEffector, oi, false),
                        new HoldWristAngle(endEffector, setpoint.wristAngle),
                        new HoldArm(arm, setpoint.armAngle)),

                //                new CloseConeGripper(endEffector),
                new AutoSetWristAngle(endEffector, Constants.EndEffector.WRIST_SAFE_ANGLE),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, idleCubeSetpoint));
    }
}
