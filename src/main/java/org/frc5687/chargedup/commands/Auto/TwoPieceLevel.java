package org.frc5687.chargedup.commands.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.frc5687.chargedup.Constants;
import org.frc5687.chargedup.commands.SnapTo;
import org.frc5687.chargedup.commands.CubeShooter.Shoot;
import org.frc5687.chargedup.subsystems.*;
import org.frc5687.chargedup.util.AutoChooser;

public class TwoPieceLevel extends SequentialCommandGroup {
    public TwoPieceLevel(
        DriveTrain driveTrain, Arm arm, Elevator elevator, EndEffector endEffector, CubeShooter shooter, AutoChooser.Node _node
    ) {
        boolean waitInstead = false;
        boolean placeCone = false;
        switch (_node) {
            case OneCone:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case TwoCube:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case ThreeCone:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case FourCone:
                placeCone = true;
                break;
            case FiveCube:
                placeCone = false;
                break;
            case SixCone:
                placeCone = true;
                break;
            case SevenCone:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case EightCube:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case NineCone:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            case Unknown:
                DriverStation.reportError("Unimplemented case: " + _node, false);
                waitInstead = true;
                break;
            default:
                DriverStation.reportError("Unexpected value: " + _node, false);
                waitInstead = true;
                break;
        }
        Command placeCommand = placeCone ? new AutoPlaceHighCone(elevator, endEffector, arm) : new AutoPlaceHighCube(elevator, endEffector, arm);
        if (waitInstead) {
            addCommands(new WaitCommand(15));
        } else {
            addCommands(
                    new SequentialCommandGroup(
                            placeCommand,
                            new LevelingAndIntake(driveTrain, shooter),
                            new SnapTo(driveTrain, new Rotation2d(Units.degreesToRadians(0))),
                            new Shoot(shooter, 1.0, Constants.CubeShooter.IDLE_ANGLE)
                    )
            );
        }
    }
}

