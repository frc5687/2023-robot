package org.frc5687.chargedup.commands.Auto;

import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePieceAuto extends SequentialCommandGroup{
    public OnePieceAuto(
        DriveTrain driveTrain,
        Arm arm,
        Elevator elevator,
        EndEffector endEffector
    ) {
        addCommands(
            new SequentialCommandGroup(
                new AutoPlaceHighCone(elevator, endEffector, arm), new DriveForTime(driveTrain, 2000)
            )
        );
    }
}
