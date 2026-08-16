# Build verification

- Use `cmake --preset MinSizeRel` whenever the project needs to be configured.
- Use `cmake --build --preset MinSizeRel` for all compilation and build verification.
- Do not compile or build with the `Debug`, `Release`, or `RelWithDebInfo` presets unless the user explicitly requests a different configuration.
