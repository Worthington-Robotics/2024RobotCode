// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.DriveController;
import frc.WorBots.subsystems.drive.Drive;
import java.util.function.Supplier;

/** Command for teleop that drives the robot using controllers */
public class DriveWithJoysticks extends Command {
  private Drive drive;
  private final DriveController driveController = new DriveController();
  private Supplier<Double> leftXSupplier;
  private Supplier<Double> leftYSupplier;
  private Supplier<Double> rightXSupplier;
  private Supplier<Double> rightYSupplier;

  public DriveWithJoysticks(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier,
      Supplier<Double> rightYSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.rightYSupplier = rightYSupplier;
  }

  @Override
  public void execute() {
    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightX = rightXSupplier.get();
    double rightY = rightYSupplier.get();

    driveController.drive(drive, leftX, leftY, rightY);
  }
}
