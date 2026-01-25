# Robot TODO List

Please ensure that you add anything left half-finished to the list, and remove anything you finished.

## Shooter
- Fix Constants.java
- Tune low/mid/high velocities
- Fix trapezoid profile constraints
- Tune feedforward as needed. kV should theoretically already be pretty close, kS definitely needs tuning
- Add low/mid/migh to RobotContainer.java
    - EX: (button) -> .onTrue -> shooter.setVelocity(lowVel);
- Make auto rpm calculator command
- Figure out inversion

## Intake
- Fix CAN ID in Constants.java
- Figure out inversion
- Test different motor power levels w/ build team