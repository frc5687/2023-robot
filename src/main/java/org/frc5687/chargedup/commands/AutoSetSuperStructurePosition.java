package org.frc5687.chargedup.commands;

import org.frc5687.chargedup.commands.Arm.AutoSetArmSetpoint;
import org.frc5687.chargedup.commands.Elevator.AutoExtendElevator;
import org.frc5687.chargedup.commands.EndEffector.AutoSetGripperAngle;
import org.frc5687.chargedup.commands.EndEffector.AutoSetWristAngle;
import org.frc5687.chargedup.commands.EndEffector.SetEndEffectorPosition;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSetSuperStructurePosition extends SequentialCommandGroup {
    private Elevator _elevator;
    private EndEffector _endEffector;
    private Arm _arm;

    private double _elevatorPosition;
    private double _wristAngle;
    private double _gripperAngle;
    private double _armAngle;

    public AutoSetSuperStructurePosition(Elevator elevator, EndEffector endEffector, Arm arm,
        double elevatorPosition, double wristAngle, double gripperAngle, double armAngle){
        _elevator = elevator;
        _endEffector = endEffector;
        _arm = arm;

        _elevatorPosition = elevatorPosition;
        _wristAngle = wristAngle;
        _gripperAngle = gripperAngle;
        _armAngle = armAngle;

        addCommands(
            new AutoSetGripperAngle(_endEffector, _gripperAngle),
            new ParallelCommandGroup(
                new AutoSetArmSetpoint(_arm, _armAngle),
                new AutoSetWristAngle(_endEffector, _wristAngle),
                new AutoExtendElevator(_elevator, _elevatorPosition)
                ),
            new AutoSetArmSetpoint(_arm, _armAngle),
            new AutoSetWristAngle(_endEffector, _wristAngle)
        );
    }
    
}
