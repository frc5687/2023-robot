package org.frc5687.chargedup.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.CubeShooter.AutoIntake;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.CubeShooter;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.util.Trajectories;

public class StealCubesAuto extends SequentialCommandGroup {
    public StealCubesAuto(
            DriveTrain drivetrain,
            EndEffector endEffector,
            Elevator elevator,
            Arm arm,
            Lights lights,
            CubeShooter _shooter,
            OI _oi,
            Trajectories trajectories) {
        addCommands(
                new Shoot(_shooter, 0.6, 0.21, _oi),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(
                                drivetrain, trajectories.getTrajectory("BUMP_GOAL_ONE"), true, true),
                        new AutoIntake(_shooter)),
                new DriveTrajectory(
                        drivetrain, trajectories.getTrajectory("GOAL_ONE_CHARGE_TWO"), true, false),
                new Shoot(_shooter, 0.6, 0.21, _oi),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(
                                drivetrain, trajectories.getTrajectory("CHARGE_TWO_GOAL_TWO"), true, false),
                        new AutoIntake(_shooter)),
                new DriveTrajectory(
                        drivetrain, trajectories.getTrajectory("GOAL_TWO_CHARGE_THREE"), true, false),
                new Shoot(_shooter, 0.6, 0.21, _oi),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(
                                drivetrain, trajectories.getTrajectory("CHARGE_THREE_GOAL_THREE"), true, false),
                        new AutoIntake(_shooter)),
                new DriveTrajectory(
                        drivetrain, trajectories.getTrajectory("GOAL_THREE_CHARGE_FOUR"), true, false),
                new Shoot(_shooter, 0.6, 0.21, _oi));
    }
}
