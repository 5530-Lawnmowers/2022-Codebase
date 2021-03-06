// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helpers.ShuffleboardHelpers;
import frc.robot.subsystems.Shooter;

public class ShooterTesting extends CommandBase {
    private final Shooter shooter;

    /**
     * Creates a new ShooterTesting.
     */
    public ShooterTesting(Shooter shooter) {
        addRequirements(shooter);
        this.shooter = shooter;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override

            //careful laptop
    public void execute() {
        shooter.setVoltage((Double) ShuffleboardHelpers.getWidgetValue("Shooter", "TopPower"),(Double) ShuffleboardHelpers.getWidgetValue("Shooter", "BotPower"));

//        shooter.setVelocityBot((Double) ShuffleboardHelpers.getWidgetValue("Shooter", "BotPower"));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setRawBot(0);
        shooter.setRawTop(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
