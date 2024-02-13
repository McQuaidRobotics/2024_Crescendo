
# Commands

## Intake

### Description

This command will move the `Stem` to the ground pickup location
and run the intake until it picks up a note or until it times out.

### Parameters

- `timeout` - The amount of time to run the intake for in seconds. Default is infinite.

## Stow

### Description

This command will move the `Stem` to the stow location and stop the intake and spin up the shooter.

### Parameters

- `rpm` - The speed to spin the shooter at in RPM. Default is 3750.

## Aim

### Description

This command will move the `Stem` to aim at the target.

### Parameters

- `angle` - The angle to aim at in degrees. Default is auto-aim.

## Shoot

### Description

Should not be used inside of a path. This command will maintain `Shooter` speed, point the `Swerve` and `Stem` at the target, and shoot the note.

### Parameters

- `rpm` - The speed to spin the shooter at in RPM. Default is 3750.

## Spinup

### Description

This command will spin up the shooter to a given speed.

### Parameters

- `rpm` - The speed to spin the shooter at in RPM. Default is 3750.