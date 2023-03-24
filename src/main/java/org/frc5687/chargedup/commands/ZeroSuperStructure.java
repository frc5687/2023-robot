package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.Arm.AutoSetArmSetpoint;
import org.frc5687.chargedup.commands.Elevator.DriveUntilInHall;
import org.frc5687.chargedup.commands.EndEffector.AutoSetWristAngle;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ZeroSuperStructure extends SequentialCommandGroup {
   Elevator _elevator;
   Arm _arm;
   EndEffector _endEffector;
   
   public ZeroSuperStructure(Elevator elevator, Arm arm, EndEffector endEffector){
    addCommands(
        new AutoSetArmSetpoint(arm, Constants.Arm.VERTICAL_ARM_ANGLE),
        new DriveUntilInHall(elevator),
        new AutoSetWristAngle(endEffector, 0.0)
    );
   }
}
