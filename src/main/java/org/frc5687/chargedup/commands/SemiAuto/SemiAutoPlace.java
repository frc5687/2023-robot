package org.frc5687.chargedup.commands.SemiAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.commands.EndEffector.HoldWristAngle;
import org.frc5687.chargedup.commands.EndEffector.WaitForManualGripper;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.util.SuperStructureSetpoints;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.highConePlaceSetpoint;
import static org.frc5687.chargedup.util.SuperStructureSetpoints.idleConeSetpoint;

public class SemiAutoPlace extends OutliersCommand {
    private Arm _arm;
    private EndEffector _endEffector;
    private Elevator _elevator;
    private OI _oi;
    public SemiAutoPlace(Arm arm, EndEffector endEffector, Elevator elevator, OI oi) {
        _arm = arm;
        _endEffector = endEffector;
        _elevator = elevator;
        _oi = oi;
    }

    @Override
    public void initialize() {
        super.initialize();
        SuperStructureSetpoints.Setpoint setpoint = _endEffector.getSuperStructureSetpoint();
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
                new AutoSetSuperStructurePosition(_elevator, _endEffector, _arm, setpoint),
                new ParallelDeadlineGroup(
                        new WaitForManualGripper(_endEffector, _oi, true),
                        new HoldWristAngle(_endEffector, setpoint.wristAngle),
                        new HoldArm(_arm, setpoint.armAngle)),
                new AutoSetRollerSpeed(_endEffector, Constants.EndEffector.PLACE_CONE_ROLLER_SPEED, true),
                new AutoSetSuperStructurePosition(_elevator, _endEffector, _arm, idleConeSetpoint));
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}