package org.frc5687.chargedup.commands.Auto;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class AutoPlaceAndStowHighCone extends SequentialCommandGroup {
    public AutoPlaceAndStowHighCone(Elevator elevator, EndEffector endEffector, Arm arm) {
        Setpoint setpoint = highConePlaceSetpoint;
        addCommands(
                new AutoSetRollerSpeed(endEffector, Constants.EndEffector.GRIPPER_IN_SPEED, false),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, setpoint),
                new AutoSetRollerSpeed(endEffector, Constants.EndEffector.PLACE_CONE_ROLLER_SPEED, true),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, idleConeSetpoint)
        );
    }
}
