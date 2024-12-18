// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.igknighters.commands.tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.igknighters.util.plumbing.TunableValues;
import com.igknighters.util.plumbing.TunableValues.TunableDouble;

public class StaticCharacterization extends Command {
  private static final TunableDouble currentRampFactor =
      TunableValues.getDouble("StaticCharacterization/CurrentRampPerSec", 1.0);
  private static final TunableDouble minVelocity =
      TunableValues.getDouble("StaticCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  public StaticCharacterization(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.value();
    inputConsumer.accept(currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.value();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Static Characterization output: " + currentInput);
    inputConsumer.accept(0);
  }
}
