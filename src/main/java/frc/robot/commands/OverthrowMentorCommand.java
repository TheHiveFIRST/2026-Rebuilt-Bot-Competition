// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Mentor;

/**
 * A command that overthrows a mentor.
 */
public class OverthrowMentorCommand extends Command {
  private final Mentor mentor;

  /**
   * Creates a new OverthrowMentorCommand.
   *
   * @param mentor The mentor to overthrow.
   */
  public OverthrowMentorCommand(Mentor mentor) {
    this.mentor = mentor;
  }

  @Override
  public void initialize() {
    mentor.overthrow();
    System.out.println(mentor.getStatus());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
