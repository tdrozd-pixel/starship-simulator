# godot_bridge (planned)

GDExtension that wraps `sim_core` and `sim_vehicle` for Godot 4. The sim runs on its own thread; this layer reads snapshots, interpolates them for each render frame, and sets node transforms and actuator angles. See "Rendering plan" in the top-level CLAUDE.md.
