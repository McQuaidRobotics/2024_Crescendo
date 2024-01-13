package com.igknighters.autos;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public interface DPBlock {
    public Supplier<Command> getCmdSupplier();
}
