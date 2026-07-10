# PathPlanner paths/autos

Empty on purpose. Open this project folder in the [PathPlanner GUI app](https://pathplanner.dev/pathplanner-getting-started.html)
and author paths/autos there against the field and this robot's actual dimensions -- that's an
interactive, field-relative design process, not something to fabricate as code.

`AutoBuilder.configure(...)` and the named event-marker commands (`Stow`, `PrepScoreMid`,
`PrepScoreHigh`, `GroundPickup`, `ChutePickup`, `ShelfPickup`, `Release` -- see
`RobotContainer.registerNamedCommands()`) are already wired up; any `.auto`/`.path` files placed
here will show up automatically in `RobotContainer`'s auto chooser and can reference those names as
event markers.
