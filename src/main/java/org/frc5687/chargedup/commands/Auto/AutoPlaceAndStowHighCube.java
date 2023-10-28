package org.frc5687.chargedup.commands.Auto;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class AutoPlaceAndStowHighCube extends SequentialCommandGroup {
    public AutoPlaceAndStowHighCube(Elevator elevator, EndEffector endEffector, Arm arm) {
        Setpoint setpoint = highCubePlaceSetpoint;
        addCommands(
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, setpoint),
                new AutoSetRollerSpeed(endEffector, Constants.EndEffector.PLACE_CUBE_ROLLER_SPEED, true),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, idleCubeSetpoint));
    }
}
