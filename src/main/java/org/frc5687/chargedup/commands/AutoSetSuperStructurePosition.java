package org.frc5687.chargedup.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.frc5687.chargedup.commands.Arm.AutoSetArmSetpoint;
import org.frc5687.chargedup.commands.Elevator.AutoExtendElevator;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.commands.EndEffector.AutoSetWristAngle;
import org.frc5687.chargedup.commands.EndEffector.SetEndEffectorPosition;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.util.SuperStructureSetpoints.Setpoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSetSuperStructurePosition extends SequentialCommandGroup {
    public AutoSetSuperStructurePosition(Elevator elevator, EndEffector endEffector, Arm arm,
        Setpoint setpoint){
        addCommands(
            // new AutoSetGripperAngle(_endEffector, _gripperAngle),
            new ParallelDeadlineGroup(
//                new AutoSetRollerSpeed(endEffector, gripperSpeed),
                new AutoSetArmSetpoint(arm, setpoint.armAngle),
                new SetEndEffectorPosition(endEffector, setpoint.wristAngle, setpoint.gripperSpeed),
                new AutoExtendElevator(elevator, setpoint.elevatorPosition)
            )
        );
    }
    
}
