package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class OuttakeCommand extends Command {
  public OuttakeCommand() {
    addRequirements(Intake.getInstance());
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
  }

  @Override
  public void execute() {
    Intake.getInstance().outtake();
  }

  @Override
  public void end(boolean interrupted) {
    Intake.getInstance().stop();
  }
}
