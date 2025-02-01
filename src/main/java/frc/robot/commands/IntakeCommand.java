package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {

  public IntakeCommand() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Intake.getInstance().intake();
  }

  @Override
  public void end(boolean interrupted) {
    Intake.getInstance().stop();
  }
}
