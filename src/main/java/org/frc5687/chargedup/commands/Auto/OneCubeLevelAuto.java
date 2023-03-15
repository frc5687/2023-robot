package org.frc5687.chargedup.commands.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.chargedup.subsystems.Arm;
import org.frc5687.chargedup.subsystems.DriveTrain;
import org.frc5687.chargedup.subsystems.Elevator;
import org.frc5687.chargedup.subsystems.EndEffector;

public class OneCubeLevelAuto extends SequentialCommandGroup {
    public OneCubeLevelAuto(
            DriveTrain driveTrain, Arm arm, Elevator elevator, EndEffector endEffector) {
        DriverStation.reportError("Starting cube and level auto", false);
        addCommands(
                new SequentialCommandGroup(
                        Commands.runOnce(endEffector::setCubeMode, endEffector),
                        new AutoPlaceHighCube(elevator, endEffector, arm),
                        new DriveUntilLevel(driveTrain)));
    }
}
