package org.frc5687.chargedup.commands.SemiAuto;

import static org.frc5687.chargedup.util.SuperStructureSetpoints.idleConeSetpoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.Arm.HoldArm;
import org.frc5687.chargedup.commands.AutoSetSuperStructurePosition;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.commands.EndEffector.AutoSetRollerSpeed;
import org.frc5687.chargedup.commands.EndEffector.HoldWristAngle;
import org.frc5687.chargedup.commands.EndEffector.WaitForPlace;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.subsystems.*;
import org.frc5687.chargedup.util.SuperStructureSetpoints;

public class SemiAutoPlace extends OutliersCommand {
    private final Arm _arm;
    private final EndEffector _endEffector;
    private final Elevator _elevator;
    private final CubeShooter _cubeShooter;
    private final DriveTrain _driveTrain;
    private final OI _oi;

    public SemiAutoPlace(
            Arm arm,
            EndEffector endEffector,
            Elevator elevator,
            CubeShooter shooter,
            DriveTrain driveTrain,
            OI oi) {
        _arm = arm;
        _endEffector = endEffector;
        _elevator = elevator;
        _cubeShooter = shooter;
        _driveTrain = driveTrain;
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
                            new WaitForPlace(_endEffector, _oi, true),
                            new HoldWristAngle(_endEffector, setpoint.wristAngle),
                            new HoldArm(_arm, setpoint.armAngle)),
                    new AutoSetRollerSpeed(_endEffector, setpoint.placeSpeed, true),
                    new AutoSetSuperStructurePosition(_elevator, _endEffector, _arm, idleConeSetpoint));
            command.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
