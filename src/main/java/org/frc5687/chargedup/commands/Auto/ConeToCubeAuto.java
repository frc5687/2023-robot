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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeToCubeAuto extends SequentialCommandGroup{

    public ConeToCubeAuto(
            DriveTrain drivetrain,
            EndEffector endEffector,
            Elevator elevator,
            Arm arm,
            Lights lights,
            CubeShooter _shooter,
            OI _oi,
            Trajectories trajectories) {
                
                String alliance = drivetrain.isRedAlliance() ? "RED_" : "BLUE_";
                addCommands(
                    new ResetRobotPose(drivetrain, new Pose2d(4.69, 13.19, new Rotation2d(0))),
                    new AutoPlaceHighCone(elevator, endEffector, arm),
                    new ParallelDeadlineGroup(
                        new DriveTrajectory(
                            drivetrain, trajectories.getTrajectory(alliance + "RED_NODE_NINE_GOAL_FOUR"), true, false),
                        new AutoIntake(_shooter)),
                    new DriveTrajectory(
                        drivetrain, trajectories.getTrajectory(alliance + "RED_GOAL_FOUR_CHARGE_THREE"), true, false),
                    new Shoot(_shooter, 1.0, Constants.CubeShooter.IDLE_ANGLE, _oi),
                    new ParallelDeadlineGroup(
                        new DriveTrajectory(
                            drivetrain, trajectories.getTrajectory(alliance + "RED_CHARGE_FOUR_GOAL_THREE"), true, true),
                            new AutoIntake(_shooter)),
                    new DriveTrajectory(
                        drivetrain, trajectories.getTrajectory(alliance + "RED_GOAL_THREE_CHARGE_FOUR"),true, true),
                    new DriveUntilLevel(drivetrain)
                    );


            }
    
}
