package org.frc5687.chargedup.commands.Auto;

import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.util.SuperStructureSetpoints.Setpoint;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.*;

import org.frc5687.chargedup.Constants;

public class AutoGroundPickupCube extends SequentialCommandGroup {
    public AutoGroundPickupCube( Elevator elevator, Arm arm, EndEffector endEffector){
        Setpoint groundSetpoint = cubeGroundPickupSetpoint;
        Setpoint idleSetpoint = idleCubeSetpoint;

        addCommands(
           new AutoSetSuperStructurePosition(elevator, endEffector, arm, groundSetpoint),
           new AutoSetRollerSpeed(endEffector, Constants.EndEffector.GRIPPER_IN_SPEED, true ),
           new AutoSetSuperStructurePosition(elevator, endEffector, arm, idleSetpoint)

        );

        }
    
}
