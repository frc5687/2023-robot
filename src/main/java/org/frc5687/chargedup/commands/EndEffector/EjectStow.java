package org.frc5687.chargedup.commands.EndEffector;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.util.SuperStructureSetpoints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class EjectStow extends SequentialCommandGroup{
   public EjectStow(EndEffector endEffector, Elevator elevator, Arm arm){
    addCommands(
        new Eject(endEffector),
        new AutoSetSuperStructurePosition(elevator, endEffector, arm, SuperStructureSetpoints.idleConeSetpoint)
    );
   } 
}
