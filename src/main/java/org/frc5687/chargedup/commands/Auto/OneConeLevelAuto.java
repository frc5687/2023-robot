package org.frc5687.chargedup.commands.Auto;

import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneConeLevelAuto extends SequentialCommandGroup{
    public OneConeLevelAuto(
        DriveTrain driveTrain,
        Arm arm,
        Elevator elevator,
        EndEffector endEffector
    ) {
        DriverStation.reportError("Starting cone and level auto", false);
        addCommands(
            new SequentialCommandGroup(
                Commands.runOnce(endEffector::setConeMode, endEffector),
                new AutoPlaceHighCone(elevator, endEffector, arm), new DriveUntilLevel(driveTrain)
            )
        );
    }
}
