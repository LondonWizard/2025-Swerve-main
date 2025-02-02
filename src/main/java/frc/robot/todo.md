# To Implement Drive Curves

1. Open `RobotContainer.java`
2. Go to `configureButtonBindings`
3. Paste the following code on line `136`, and delete the two function calls that used to be there:

```java
    final CurvedController curvedController = new CurvedController(controller);
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -curvedController.curvedTranslationX(),
            () -> -curvedController.curvedTranslationY(),
            () -> -curvedController.curvedRotation()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -curvedController.curvedTranslationX(),
                () -> -curvedController.curvedTranslationY(),
                () -> new Rotation2d()));
```
