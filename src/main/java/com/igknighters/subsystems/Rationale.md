# Code structure rationale

## Cooperation inside team

By fragmenting the hardware interface to this degree it allows each programmer to have
their own subsystem to work on without having to worry about stepping on each other's toes.
Each programmer still gets experience working with other programmers' code through the java interfaces.

## Separation of concerns

Finding where the code for a certain piece of hardware on the robot is located is easy.
Each subsystem is in its own file, and each subsystem is in its own package with its components.
This makes it easy to find the code for a certain subsystem, and it makes it easy to find the code for a certain component.

## Proper amount of abstraction

The interfaces allow for sim code and non-sim code to be separated.
The stem and umbrella being separate allows for simpler commands to be written
to just control one or the other versus both.

## Easy to extend

Mechanical is always talking about adding more to the robot.
Setting the precedent of sub-components allows for the incremental development of new components
with an easier way of disabling them.
