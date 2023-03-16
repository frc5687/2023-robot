package org.frc5687.chargedup.commands.SemiAuto;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.commands.EndEffector.HoldWristAngle;
import org.frc5687.chargedup.commands.EndEffector.WaitForPlace;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class SemiAutoPlaceMiddleCube extends SequentialCommandGroup {
    public SemiAutoPlaceMiddleCube(Arm arm, EndEffector endEffector, Elevator elevator, OI oi) {
        Setpoint setpoint = middleCubePlaceSetpoint;
        addCommands(
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, setpoint),
                new ParallelDeadlineGroup(
                        new WaitForPlace(endEffector, oi, true),
                        new HoldWristAngle(endEffector, setpoint.wristAngle),
                        new HoldArm(arm, setpoint.armAngle)),
                new AutoSetRollerSpeed(endEffector, Constants.EndEffector.PLACE_CUBE_ROLLER_SPEED, true),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, idleCubeSetpoint));
    }
}
