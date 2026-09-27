# TeamCode organization

Robot code belongs under `src/main/java/org/firstinspires/ftc/teamcode`.

```text
teamcode/
├── actions/                 Reusable, nonblocking robot behaviors
├── autonomous/
│   ├── paths/               Pedro path and pose definitions
│   └── routines/            Autonomous sequences built from paths and actions
├── config/                  Robot-wide names, limits, and setpoints
├── control/                 Gamepad mappings and TeleOp subsystem control
├── opmodes/
│   ├── autonomous/          Competition autonomous entry points
│   ├── teleop/              Competition driver-controlled entry points
│   └── tests/               Test, calibration, and diagnostic OpModes
├── pedro/                   Pedro configuration and tuning supplied by Quickstart
├── subsystems/              Hardware-owning classes, grouped by mechanism
└── util/                    Small helpers with no mechanism ownership
```

Keep registered OpModes thin. They should select and coordinate behavior while
subsystems own hardware access and mechanism state. Add a child package under
`subsystems` when a mechanism is known instead of creating speculative mechanism
folders at the start of the season.

