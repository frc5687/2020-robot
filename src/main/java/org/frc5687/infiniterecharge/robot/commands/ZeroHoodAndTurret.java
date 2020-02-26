package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;

public class ZeroHoodAndTurret extends SequentialCommandGroup {
    public ZeroHoodAndTurret(Hood hood, Turret turret) {
        addCommands(
        new ZeroSensors(hood, turret),
        new AutoTurretSetpoint(turret, 0));
    }
}