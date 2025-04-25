# Panda3D FPS Demo Script (zeta.py)

Version: 0.01 (As of April 26, 2025)
Location: Brisbane, Queensland, Australia

## Description

This script demonstrates various features within the Panda3D engine for creating a basic first-person shooter (FPS) style game prototype. It includes player movement, physics interactions using the Bullet engine, simple enemy AI, attack mechanics, and procedural visual elements.

## Features

* **Player:**
    * First-person camera perspective.
    * WASD movement controls (physics-based using velocity).
    * Spacebar jump (physics-based impulse).
    * Mouse look (smoothed with deadzone).
    * Visual "blades" attached to the camera view.
    * Melee attack (Left Mouse): Animates left blade, performs a distance check (2 units range) to despawn attackers.
    * Ranged "Spit" Attack (Right Mouse): Fires a projectile (billboarded sprite) that travels forward.
    * Basic health system (initialized but not yet depleted by enemies).
* **Attackers (`ATTACKERS` Class):**
    * Procedurally generated visuals (stack of randomly rotated, alternating pink/yellow plates).
    * Simple sphere physics collision shape.
    * Basic AI: Paths towards the center (0,0,0) initially. Switches to chasing the player if the player enters a 3-unit radius.
    * Despawn logic: Despawns if it reaches the center barrier or if hit by player melee/spit attacks.
    * Respawn logic: Schedules 2 new attackers to spawn 1-3 seconds after one despawns.
* **Environment:**
    * Physics world using Panda3D's Bullet integration.
    * Infinite physics ground plane at Z=0.
    * Large visual ground plane with tiled texture (`trip-floor.jpg`).
    * 13-sided polygon physics wall boundary (Radius 50) with texture (`wall-blob.jpg`).
    * Central barrier object made of 3 stacked, rotated physics spheres.
    * Basic ambient and directional lighting.
* **Game Logic:**
    * Failure Condition: Game resets if 10 attackers reach the center within 3 seconds (includes screen flash).
    * Game Reset: Resets player health/position, clears attackers, removes pending respawns, spawns one initial attacker.
    * ESC key exits the application.
* **Technical:**
    * Uses Panda3D task manager for updates.
    * Uses Panda3D Interval system for attack animation.
    * Uses Bullet collision events (`bullet-contact-added`) for projectile hit detection.
    * Uses Python tags for identifying physics objects in collisions.
    * Configured for threaded rendering (`Cull/Draw`).
    * Physics debug visualization is enabled (shows wireframe collision shapes).

## Requirements

* Python 3 (Developed using 3.13)
* Panda3D SDK (Developed using 1.10.15) - Install via `pip install panda3d`
* Image Files (Place in the same directory as the script, or update paths):
    * `trip-floor.jpg` (For the ground texture)
    * `wall-blob.jpg` (For the polygon wall texture)
    * `acid_sprite.png` (For the projectile visual - needs transparency)

## How to Run

1.  Make sure Python and Panda3D are installed.
2.  Ensure the required image files (`trip-floor.jpg`, `wall-blob.jpg`, `acid_sprite.png`) are accessible to the script.
3.  Save the script (e.g., as `zeta.py`).
4.  Run from the command line: `python zeta.py`

## Controls

* **W, A, S, D:** Move Forward / Strafe Left / Move Backward / Strafe Right
* **Space:** Jump
* **Mouse:** Look Around
* **Left Mouse Button:** Melee Attack
* **Right Mouse Button:** Spit Attack
* **ESC:** Exit Application

## Known Issues / Areas for Refinement

* **Mouse Confinement:** The relative mouse mode (`M_relative`) used for FPS controls may not perfectly confine the mouse cursor to the window on some multi-monitor setups (like the one used during development on Windows 10). OS-level workarounds might be necessary. `#TODO: Investigate OS-specific mouse confinement issues.`
* **Potential Mouse Fix (User Provided):** The developer found the following link helpful for their specific multi-monitor mouse issue involving diagonal screen positioning in Windows Display Settings, though its content and general applicability cannot be verified: `https://www.youtube.com/watch?v=VulzoJgoMI0`
* **Attacker Pathfinding:** The current AI is very basic. Attackers move directly towards their target (center or player) without considering obstacles. Pathfinding could be improved using navigation meshes or steering behaviors. The chasing logic is also simple. `#TODO: Refine Attacker Pathfinding/AI`
* **Spit Attack Collision/Damage:** The projectile currently despawns itself and the attacker instantly on contact via collision events. This works, but could be refined: `#TODO: Refine Projectile hit effects (damage, visuals, sounds)`
* **Melee Attack Hit Detection:** The melee attack uses a simple distance check from a point in front of the player. This could be improved using a shape test (like `convexSweepTest` or a ghost node) for more accurate volume detection. `#TODO: Refine Melee hit detection`
* **Player Damage:** The player health variable exists, but attackers do not currently damage the player upon collision. This needs to be implemented. `#TODO: Implement player damage from attackers.`
* **Round Progression:** The logic for ending a round when all attackers are cleared and starting a new, potentially harder round is not implemented. `#TODO: Implement round progression.`
* **Visuals:** Models used for blades and attackers are placeholders (`xyzAxis`, stacked boxes). These could be replaced with custom models.
