package org.frc5687.chargedup.commands.Auto;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class AutoPlaceHighCube extends SequentialCommandGroup {
    public AutoPlaceHighCube(Arm arm, EndEffector endEffector, Elevator elevator) {
        Setpoint setpoint = highCubePlaceSetpoint;
        addCommands(
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, setpoint),
                new AutoSetRollerSpeed(endEffector, Constants.EndEffector.PLACE_CUBE_ROLLER_SPEED, true),
                new AutoSetSuperStructurePosition(elevator, endEffector, arm, idleCubeSetpoint));
    }
}
