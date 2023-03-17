package org.frc5687.chargedup.commands.Auto;

import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.CubeShooter;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.commands.OutliersCommand;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StealCubesAuto extends SequentialCommandGroup{
    public StealCubesAuto(
        DriveTrain drivetrain,
        EndEffector endEffector,
        Elevator elevator,
        Arm arm,
        Lights lights,
        CubeShooter _shooter,
        OI _oi
    ) {
        addCommands(
            new Shoot(_shooter, 0, 0, _oi), //change these values
            new DriveTrajectory(drivetrain, /*start to first cube*/),
            new DriveTrajectory(drivetrain, /*first cube to in front of charge station lined up with first cube*/),
            new Shoot(_shooter, 0, 0, _oi), //change these values
            new DriveTrajectory(drivetrain, /*charge station to second cube*/),
            new DriveTrajectory(drivetrain, /*second cube to in front of charge station lined up with second cube*/),
            new Shoot(_shooter, 0, 0, _oi), //change these values
            new DriveTrajectory(drivetrain, /*charge station to third cube*/),
            new DriveTrajectory(drivetrain, /*third cube to in front of charge station lined up with second cube*/),
            new Shoot(_shooter, 0, 0, _oi) //change these values
        );
    }
}
