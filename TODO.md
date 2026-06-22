- Revisit servo and motor configuration
- Create a system where multiple versions of some tasks can be spawned and
  ensure that others only get spawned once
  - When implementing this we must also pass as arguments all sync primitives
    instead of statically defining them (required for tasks that are spawned
    multiple times)
