## Never use LazyLock in interrupt executors!!

This is the story of a very confusing dead-lock. I was using a `LazyLock` to make tasks wait until the config was read from flash. Big mistake (at least when you have executors at multiple priority levels)! Aparently, the `LazyLock.get()` future doesn't actually put the task to sleep, but instead just rewakes the task immediately if the data isn't available. This means that the high priority executor was running in an infinite loop, consuming all CPU time and preventing the config from being read in a lower priority executor. Don't forget to make the awaited task have the highest priority.

## When the buffered uart interrupt priority is less than the uart reader task, you get a deadlock (when an error occurs?)

## Investigate why the battery voltage is not being sent over telemetry

## Got dshot esc working by sending command 0 (STOP) when disarmed

- TODO: Bad abstractions for motors in control (change configuration parameters etc.)

## Imu data is very weird after sensor to body transform

- Apparently the second flight controller didn't have the accelerometer axes reversed
