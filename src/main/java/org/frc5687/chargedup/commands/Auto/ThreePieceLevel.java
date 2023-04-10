package org.frc5687.chargedup.commands.Auto;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.OI;
import org.frc5687.chargedup.commands.DriveTrajectory;
import org.frc5687.chargedup.commands.CubeShooter.AutoIntake;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.CubeShooter;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;
import org.frc5687.chargedup.subsystems.Lights;
import org.frc5687.chargedup.util.Trajectories;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreePieceLevel extends SequentialCommandGroup {
    public ThreePieceLevel(
        DriveTrain drivetrain,
        Arm arm,
        Elevator elevator,
        EndEffector endEffector,
        Lights lights,
        OI oi,
        CubeShooter shooter,
        Trajectories trajectories){
        String alliance = drivetrain.isRedAlliance()? "RED_":"BLUE_" ;
            addCommands(
                new ResetRobotPose(drivetrain, trajectories.getTrajectory(alliance + "BUMP_GOAL_ONE").getInitialHolonomicPose()),
                new Shoot(shooter, 1.0, Constants.CubeShooter.IDLE_ANGLE, oi),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(
                                drivetrain, trajectories.getTrajectory(alliance + "BUMP_GOAL_ONE"), true, false),
                        new AutoIntake(shooter, false, oi)),
                new DriveTrajectory(
                        drivetrain, trajectories.getTrajectory(alliance + "GOAL_ONE_CHARGE_TWO"), true, false),
                new Shoot(shooter, 1.0, Constants.CubeShooter.IDLE_ANGLE, oi),
                new ParallelDeadlineGroup(
                        new DriveTrajectory(
                                drivetrain, trajectories.getTrajectory(alliance + "CHARGE_TWO_GOAL_TWO"), true, false),
                        new AutoIntake(shooter, false, oi)),
                new DriveTrajectory(
                        drivetrain, trajectories.getTrajectory(alliance + "GOAL_TWO_CHARGE_THREE"), true, false),
                new Shoot(shooter, 1.0, Constants.CubeShooter.IDLE_ANGLE, oi),
                new QuickLevel(drivetrain, false) 
            );
    }  
}
