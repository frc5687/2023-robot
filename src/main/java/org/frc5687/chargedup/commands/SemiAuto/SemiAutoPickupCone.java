package org.frc5687.chargedup.commands.SemiAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.Arm.AutoSetArmSetpoint;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.Elevator.AutoExtendElevator;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.commands.EndEffector.AutoSetWristAngle;
import org.frc5687.chargedup.commands.EndEffector.CloseConeGripper;
import org.frc5687.chargedup.commands.EndEffector.WaitForManualGripper;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

public class SemiAutoPickupCone extends SequentialCommandGroup {
    public SemiAutoPickupCone(
            Arm arm,
            EndEffector endEffector,
            Elevator elevator,
            OI oi
        ) {
        Setpoint setpoint = conePickupSetpoint;
        addCommands(
                // new DriveUntilInHall(elevator),
               // new AutoExtendElevator(elevator, 0.01),
                new AutoSetWristAngle(endEffector, Constants.EndEffector.WRIST_SAFE_ANGLE),
                new AutoExtendElevator(elevator, setpoint.elevatorPosition),
                new ParallelDeadlineGroup(new AutoSetArmSetpoint(arm, setpoint.armAngle), new AutoSetRollerSpeed(endEffector, setpoint.gripperSpeed, isFinished())),
                new ParallelDeadlineGroup(new WaitForManualGripper(endEffector, oi, false), new HoldArm(arm, setpoint.armAngle)),

//                new CloseConeGripper(endEffector),
                new AutoSetWristAngle(endEffector, idleConeSetpoint.wristAngle),
                new ParallelDeadlineGroup(new AutoSetArmSetpoint(arm, idleConeSetpoint.armAngle), new AutoSetRollerSpeed(endEffector, idleConeSetpoint.gripperSpeed, isFinished()))
        );
    }
}
