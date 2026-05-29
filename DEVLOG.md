## Never use LazyLock in interrupt executors!!

This is the story of a very confusing dead-lock. I was using a `LazyLock` to make tasks wait until the config was read from flash. Big mistake (at least when you have executors at multiple priority levels)! Aparently, the `LazyLock.get()` future doesn't actually put the task to sleep, but instead just rewakes the task immediately if the data isn't available. This means that the high priority executor was running in an infinite loop, consuming all CPU time and preventing the config from being read in a lower priority executor.
