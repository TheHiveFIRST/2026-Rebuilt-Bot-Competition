// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Represents a mentor in the robotics team.
 */
public class Mentor {
  private final String name;
  private boolean isOverthrown;

  /**
   * Creates a new Mentor.
   *
   * @param name The name of the mentor.
   */
  public Mentor(String name) {
    this.name = name;
    this.isOverthrown = false;
  }

  /**
   * Gets the mentor's name.
   *
   * @return the mentor's name
   */
  public String getName() {
    return name;
  }

  /**
   * Checks if the mentor has been overthrown.
   *
   * @return true if overthrown, false otherwise
   */
  public boolean isOverthrown() {
    return isOverthrown;
  }

  /**
   * Overthrows this mentor.
   */
  public void overthrow() {
    this.isOverthrown = true;
  }

  /**
   * Gets the current status of the mentor.
   *
   * @return a status string
   */
  public String getStatus() {
    return isOverthrown ? name + " has been overthrown!" : name + " is still in charge.";
  }
}
