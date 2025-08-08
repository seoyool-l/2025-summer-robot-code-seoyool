package org.blackknights.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.blackknights.subsystems.IntakeSubsystem;
import org.blackknights.subsystems.ShooterSubsystem;
import org.blackknights.constants.ShooterConstants;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    @Override
    public void execute() {
        shooterSubsystem.setBothMotorPercent(ShooterConstants.MAX_SPEED);
    }
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setBothMotorPercent(0.0);
    }
}
