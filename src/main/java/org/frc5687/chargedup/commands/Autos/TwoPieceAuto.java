package org.frc5687.chargedup.commands.Autos;

import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoPieceAuto extends SequentialCommandGroup{
    private Rotation2d _rotation1;
    private Rotation2d _rotation2;

    private Trajectory _trajectory1;
    private Trajectory _trajectory2;

    public TwoPieceAuto(
        DriveTrain driveTrain,
        EndEffector gripper,
        EndEffector wrist,
        Elevator elevator,
        Arm arm
       // Lights lights
    ) {
     addCommands(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
//             new AutoPlaceGamePiece()    
            )   
        )
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(driveTrain, _trajectory1, _rotation1)
            )    
            //new GroundPickupGamePiece()    
        )
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(driveTrain, _trajectory2, _rotation2)
            ) //new AutoPlaceGamePiece()
        )
     );
    }
    

    
}
