(fyi, no solder is needed, it is all plug-and play, with a bit of coding magic of course)

Control Hub

Motor ports:

Port 0 - fl - Front Left drive, GoBilda 5203 312RPM, REVERSED

Port 1 - fr - Front Right drive, GoBilda 5203 312RPM, FORWARD

Port 2 - bl - Back Left drive, GoBilda 5203 312RPM, REVERSED

Port 3 - br - Back Right drive, GoBilda 5203 312RPM, FORWARD

Servo ports:

Port 0 - stopper - ball stopper, REV Smart Servo

Port 1 - pto - PTO actuator, REV Smart Servo, inverted

Port 2 - hood - Flywheel hood angle, REV Smart Servo

I2C ports:

Port 1 - pinpoint — GoBilda Pinpoint Odometry Computer, address 0x31
USB:

Port 0 AprilTag camera (used by AprilTagTracking)
Expansion Hub

Motor ports:

Port 0 - turret - Turret rotation, GoBilda 5203 60RPM, inverted via setInverted(true)

Port 1 - intake - Intake/PTO motor, GoBilda 5203 312RPM, REVERSED

Port 2 - rsh - Right flywheel, GoBilda 5203 (high RPM variant), FORWARD

Port 3 - lsm - Left flywheel, GoBilda 5203 (high RPM variant), FORWARD

Analog input ports:

Port 0 - s1 - Intake ball sensor 1, SWYFT analog distance sensor

Port 1 - s2 - Intake ball sensor 2, SWYFT analog distance sensor

Port 2 - s3 - Intake ball sensor 3, SWYFT analog distance sensor
