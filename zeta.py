# Panda3D FPS Demo Script - zeta.py
# Date: Saturday, April 26, 2025
# Location: Brisbane, Queensland, Australia
# Current Version: 0.01
#
# Features:
# - FPS Controls (WASD/Mouse/Jump) with simple Blades visual.
# - Bullet Physics for Player, Attackers, Environment.
# - Infinite Ground Plane with Tiled Texture.
# - 13-Sided Polygon Wall boundary.
# - Stacked Sphere central barrier object.
# - Basic Attacker AI (Path to center, Chase player within range).
# - Player Melee Attack (Left Click) - Simple distance check hit.
# - Player Ranged Attack (Right Click) - Projectile with basic collision.
# - Attacker Respawn (2 spawn 1-3s after 1 despawns).
# - Failure/Reset Condition (10 attackers reach center within 3s).
# - ESC key to exit.
# - Attacker Visual: Stacked boxes with alternating colors.

import sys
import math
import random
import panda3d
import collections # For deque
from direct.showbase.ShowBase import ShowBase
from direct.task import Task # For task constants (Task.cont, Task.done)
from panda3d.core import (
    loadPrcFileData,
    Vec3, Point3, BitMask32, Quat, VBase3, Vec4, # Core types
    AmbientLight, DirectionalLight,             # Lighting
    WindowProperties,                           # Window control
    NodePath,                                   # Scene graph nodes
    PandaNode,
    CardMaker,                                  # For ground visual / projectile sprite
    Texture,                                    # Textures
    TextureStage,                               # Texture properties
    Filename,                                   # OS-specific paths
    TransformState,                             # For physics shape offsets
    TransparencyAttrib                          # For projectile sprite alpha
)
# Import the bullet module itself for physics
import panda3d.bullet
# Explicit bullet shape imports for clarity
from panda3d.bullet import BulletSphereShape, BulletBoxShape, BulletPlaneShape, BulletCapsuleShape

# Import Interval system for animations
from direct.interval.IntervalGlobal import Sequence, LerpPosInterval, Func

# --- Print Panda3D Version ---
print("Using Panda3D version:", panda3d.__version__)

# --- Panda3D Configuration ---
loadPrcFileData("", "threading-model Cull/Draw") # Use threads for rendering stages
#loadPrcFileData("", "fullscreen #t")             # Uncomment for fullscreen
loadPrcFileData("", "win-size 1366 768")       # Set window size
# loadPrcFileData("", "raw-mice #t")             # Uncomment to try raw mouse input if confinement fails

# --- Collision Masks ---
# Using bitmasks to filter physics collisions between different object types
COLLISION_MASK_GROUND = BitMask32(0x1)
COLLISION_MASK_PLAYER = BitMask32(0x2)
COLLISION_MASK_WALL = BitMask32(0x4)
COLLISION_MASK_ATTACKER = BitMask32(0x8)
COLLISION_MASK_SPHERE = BitMask32(0x10)
COLLISION_MASK_PROJECTILE = BitMask32(0x20)


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ PROJECTILE CLASS +++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class Projectile:
    # Represents a single spit attack projectile.
    projectile_radius = 0.2   # Physics collision radius
    projectile_speed = 50.0   # Units per second
    projectile_lifetime = 3.0 # Seconds before self-destructing

    def __init__(self, app, start_pos, direction):
        # Initializes the projectile's physics, visuals, and lifetime.
        self.app = app          # Reference to the main application
        self.nodepath = None    # Will hold the main NodePath for the projectile

        # --- Visual Billboarded Sprite ---
        cm = CardMaker("projectile_visual")
        # Make card slightly larger than physics radius for visibility
        card_size = self.projectile_radius * 3
        cm.setFrame(-card_size, card_size, -card_size, card_size)
        cm.setUvRange((0,0), (1,1))
        self.visual_np = NodePath(cm.generate())
        try:
            # Requires 'acid_sprite.png' (ideally with alpha transparency)
            tex = self.app.loader.loadTexture("acid_sprite.png")
            tex.setWrapU(Texture.WM_clamp); tex.setWrapV(Texture.WM_clamp) # Don't tile sprite
            self.visual_np.setTexture(tex)
            self.visual_np.setTransparency(TransparencyAttrib.MAlpha) # Use texture's alpha channel
        except Exception as e:
            print(f"WARN: Could not load projectile texture: {e}. Using color.")
            self.visual_np.setColor(0.5, 1, 0.5, 0.8) # Fallback green blob
        # Make the card always face the camera
        self.visual_np.setBillboardPointEye()

        # --- Physics ---
        shape = BulletSphereShape(self.projectile_radius)
        body_node_name = f'Projectile_{id(self)}' # Unique name
        self.body_node = panda3d.bullet.BulletRigidBodyNode(body_node_name)
        self.body_node.addShape(shape)
        self.body_node.setMass(0.1) # Small mass, needed for CCD
        self.body_node.setLinearSleepThreshold(0) # Don't sleep
        # Continuous Collision Detection (CCD) for fast-moving objects
        self.body_node.setCcdMotionThreshold(1e-7)
        self.body_node.setCcdSweptSphereRadius(self.projectile_radius)
        self.body_node.setGravity(Vec3(0,0,0)) # Projectile ignores gravity
        self.body_node.notifyCollisions(True) # Generate 'bullet-contact-added' events

        # --- NodePath and Scene Graph ---
        self.nodepath = self.app.render.attachNewNode(self.body_node)
        self.nodepath.setPos(start_pos)
        self.visual_np.reparentTo(self.nodepath) # Attach visual sprite to physics node

        # --- Collision Setup ---
        # This object *IS* a projectile
        self.nodepath.node().setIntoCollideMask(COLLISION_MASK_PROJECTILE)
        # It should check for collisions with these types
        self.nodepath.setCollideMask( COLLISION_MASK_WALL | COLLISION_MASK_ATTACKER |
                                      COLLISION_MASK_GROUND | COLLISION_MASK_SPHERE )
        # Tagging for identification in collision handler
        self.nodepath.setPythonTag("type", "projectile")
        self.nodepath.setPythonTag("object", self) # Reference to this instance

        # --- Finalize ---
        self.app.physics_world.attachRigidBody(self.body_node)
        # Apply initial velocity
        self.body_node.setLinearVelocity(direction * self.projectile_speed)
        # Schedule automatic removal after lifetime
        self.lifetime_task_name = f'ProjectileLifetime_{id(self)}'
        self.app.taskMgr.doMethodLater(self.projectile_lifetime, self.despawn, self.lifetime_task_name)

    def despawn(self, task=None):
        # Safely removes the projectile from the physics world and scene graph.
        self.app.taskMgr.remove(self.lifetime_task_name) # Prevent duplicate calls
        if self.nodepath is not None and not self.nodepath.isEmpty():
            # print(f"Despawning Projectile {self.body_node.getName()}") # Optional log
            self.app.physics_world.removeRigidBody(self.body_node)
            self.nodepath.removeNode()
            self.nodepath = None # Clear reference
        return Task.done # Required return value for tasks
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ END PROJECTILE CLASS +++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ ATTACKER CLASS (Alternating Layer Colors) +++ <<< MODIFIED
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class ATTACKERS:
    # Represents an enemy attacker unit.
    # Class variables for properties
    attacker_radius = 0.7 # Used for physics sphere size and Z pos target
    visual_base_radius = attacker_radius * 0.9 # Base size for visual layers
    physics_radius = attacker_radius # Radius of the physics collision sphere
    # --- Define alternating colors --- <<< ADDED
    color1 = Vec4(1.0, 0.75, 0.8, 1) # Pink
    color2 = Vec4(1.0, 1.0, 0.5, 1) # Yellow
    # --------------------------------

    def __init__(self, app, start_pos):
        # Initializes physics, visuals (with alternating colors), and state for the attacker.
        self.app = app; self.move_speed = 4.0; self.state = "path_to_center";
        self.target_pos = Point3(0, 0, self.physics_radius); self.chase_radius_sq = 3.0 * 3.0

        # Physics Setup
        shape = BulletSphereShape(self.physics_radius); body_node_name = f'Attacker_{id(self)}';
        self.body_node = panda3d.bullet.BulletRigidBodyNode(body_node_name); self.body_node.addShape(shape);
        self.body_node.setMass(1.0); self.body_node.setLinearSleepThreshold(0); self.body_node.setAngularFactor(Vec3(0,0,0))

        # NodePath and Scene Graph
        self.nodepath = self.app.render.attachNewNode(self.body_node); self.nodepath.setPos(start_pos)
        # Collision Setup & Tagging
        self.nodepath.node().setIntoCollideMask(COLLISION_MASK_ATTACKER)
        self.nodepath.setCollideMask( COLLISION_MASK_GROUND | COLLISION_MASK_PLAYER | COLLISION_MASK_SPHERE | COLLISION_MASK_ATTACKER | COLLISION_MASK_PROJECTILE )
        self.nodepath.setPythonTag("type", "attacker"); self.nodepath.setPythonTag("object", self)

        # --- Generated Visual Setup (Stack of Boxes with Alternating Colors) --- <<< MODIFIED
        self.visual_model = NodePath(f"attacker_visual_{id(self)}")
        base_shape = self.app.loader.loadModel("models/box") # Assumed 2x2x2

        num_layers = random.randint(15, 40)
        total_height = self.app.player_height * 1.5
        layer_height = total_height / num_layers if num_layers > 0 else total_height

        for i in range(num_layers):
            layer_np = self.visual_model.attachNewNode(f"layer_{i}")
            base_shape.instanceTo(layer_np)

            layer_z = (i * layer_height) - (total_height / 2.0) + (layer_height / 2.0) + self.physics_radius
            layer_h = random.uniform(0, 360)
            layer_r = self.visual_base_radius * random.uniform(0.7, 1.0)

            # Scale includes Z height now
            layer_np.setScale(layer_r, layer_r, layer_height / 2.0)
            layer_np.setPos(0, 0, layer_z)
            layer_np.setH(layer_h)

            # --- Apply Alternating Color --- <<< MODIFIED
            if i % 2 == 0:
                layer_np.setColor(self.color1) # Use color1 (Pink)
            else:
                layer_np.setColor(self.color2) # Use color2 (Yellow)
            # -------------------------------

        self.visual_model.reparentTo(self.nodepath) # Attach completed visual to physics node
        # ----------------------------------------------------------------------------

        # Finalize
        self.app.physics_world.attachRigidBody(self.body_node); print(f"Spawned Attacker {body_node_name} at {start_pos} with {num_layers} layers.")

    def update(self, dt):
        # Runs AI logic and movement for the attacker each frame.
        # (Update logic remains the same)
        if not self.nodepath or self.nodepath.isEmpty(): return
        self.body_node.setActive(True, True); current_pos = self.nodepath.getPos(self.app.render); current_vel = self.body_node.getLinearVelocity()
        player_pos = self.app.player_np.getPos(self.app.render); vector_to_player = player_pos - current_pos; dist_sq_to_player = vector_to_player.lengthSquared()
        if dist_sq_to_player < self.chase_radius_sq: self.state = "chasing"; self.target_pos = player_pos
        else:
            if self.state != "chasing": self.state = "path_to_center"; self.target_pos = Point3(0, 0, self.physics_radius)
        if self.state == "path_to_center":
            contact_result = self.app.physics_world.contactTestPair(self.body_node, self.app.barrier_sphere_phys_node)
            if contact_result.getNumContacts() > 0: print(f"Attacker {self.body_node.getName()} contacted center sphere."); self.app.attacker_reached_center(); self.despawn(); return
        direction_to_target = self.target_pos - current_pos; direction_to_target.z = 0
        if direction_to_target.lengthSquared() > 1e-4:
             direction_to_target.normalize(); target_vel = Vec3(direction_to_target.x * self.move_speed, direction_to_target.y * self.move_speed, current_vel.z); self.body_node.setLinearVelocity(target_vel)
        else: self.body_node.setLinearVelocity(Vec3(0, 0, current_vel.z))

    def despawn(self):
        # Removes the attacker and schedules TWO new attackers to spawn.
        # (Despawn logic remains the same)
        if self.nodepath is not None and not self.nodepath.isEmpty():
            attacker_name = self.body_node.getName(); print(f"Despawning Attacker {attacker_name}")
            delay = random.uniform(1.0, 3.0); task_name = f'RespawnAttackerTask_{id(self)}'
            self.app.taskMgr.doMethodLater(delay, self.app.spawn_two_attackers, task_name); print(f"  Scheduled 2 respawns in {delay:.2f} seconds.")
            self.app.physics_world.removeRigidBody(self.body_node); self.nodepath.removeNode(); self.nodepath = None
            if self in self.app.attackers_list: self.app.attackers_list.remove(self)
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ END ATTACKER CLASS +++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# --- Main Application Class ---
class PhysicsWorldApp(ShowBase):
    # Main application class.

    def __init__(self):
        # Initializes the application, sets up the scene, physics, player, and tasks.
        ShowBase.__init__(self); self.disableMouse(); self.setBackgroundColor(0.1, 0.1, 0.15)
        self.sphere_radius = 5.0; self.wall_radius = 50.0; self.player_height = 1.8
        self.setup_lighting(); self.setup_physics_and_ground()
        self.setup_polygon_wall(num_sides=13, radius=self.wall_radius)
        self.barrier_sphere_phys_node, self.barrier_sphere_np = self.setup_barrier_sphere(self.sphere_radius)
        self.player_phys_node, self.player_np = self.setup_player(radius=0.4, height=self.player_height)
        self.camera.reparentTo(self.player_np); self.camera.setPos(0, 0, 1.5)
        # Minimap removed
        self.setup_blades(); self.attack_sequence = None; self.is_attacking = False
        self.player_health = 3; print(f"Player health initialized to: {self.player_health}")
        self.recent_center_touches = collections.deque(); self.failure_time_window = 3.0; self.failure_touch_count = 10; self.game_over = False; self.original_bg_color = self.getBackgroundColor()
        self.attackers_list = []; self.spawn_attacker()
        self.key_map = {"forward": False, "backward": False, "left": False, "right": False, "jump": False}
        self.setup_input(); self.accept('bullet-contact-added', self.on_bullet_contact)
        self.taskMgr.add(self.update_physics, "update_physics_task", sort=1); self.taskMgr.add(self.update_player, "update_player_task", sort=2); self.taskMgr.add(self.mouse_look_task, "mouse_look_task", sort=3); self.taskMgr.add(self.update_attackers, "update_attackers_task", sort=4)
        print("------------------------------------"); print("Panda3D FPS Demo Initialized") # ...rest of prints...

    def setup_lighting(self):
        # Sets up scene lighting.
        ambient_light=AmbientLight('al');ambient_light.setColor((.3,.3,.3,1));alnp=self.render.attachNewNode(ambient_light);self.render.setLight(alnp); dir_light=DirectionalLight('dl');dir_light.setColor((.8,.8,.7,1));dlnp=self.render.attachNewNode(dir_light);dlnp.setHpr(30,-60,0);self.render.setLight(dlnp); dlnp2=self.render.attachNewNode(dir_light);dlnp2.setHpr(-30,-60,0);self.render.setLight(dlnp2)

    def setup_physics_and_ground(self):
        # Sets up physics world, ground plane (physics and visual), and debug view.
        # (Includes collision masks + tag)
        self.physics_world = panda3d.bullet.BulletWorld(); self.physics_world.setGravity(Vec3(0, 0, -9.81))
        ground_shape = panda3d.bullet.BulletPlaneShape(Vec3(0, 0, 1), 0); ground_node = panda3d.bullet.BulletRigidBodyNode('Ground'); ground_node.addShape(ground_shape); ground_node.setMass(0)
        ground_phys_np = self.render.attachNewNode(ground_node); ground_phys_np.node().setIntoCollideMask(COLLISION_MASK_GROUND); ground_phys_np.setCollideMask(COLLISION_MASK_PLAYER | COLLISION_MASK_ATTACKER | COLLISION_MASK_PROJECTILE); ground_phys_np.setPythonTag("type", "ground"); ground_phys_np.setPythonTag("object", True)
        self.physics_world.attachRigidBody(ground_node); print("Added static ground plane physics at Z=0")
        ground_visual_radius = self.wall_radius * 1.1; ground_size = ground_visual_radius * 2.0; half_size = ground_size / 2.0; cm = CardMaker("ground_visual"); cm.setFrame(-half_size, half_size, -half_size, half_size); cm.setUvRange((0, 0), (1, 1)); self.ground_visual_np = self.render.attachNewNode(cm.generate()); self.ground_visual_np.setPos(0, 0, 0.01); self.ground_visual_np.setP(-90); print(f"Added visual ground plane (Size: {ground_size}x{ground_size})")
        try: texture_path = "trip-floor.jpg"; tex = self.loader.loadTexture(Filename.from_os_specific(texture_path)); tex.setWrapU(Texture.WM_repeat); tex.setWrapV(Texture.WM_repeat); self.ground_visual_np.setTexture(tex); tile_repeat_x = ground_size; tile_repeat_y = ground_size; self.ground_visual_np.setTexScale(TextureStage.getDefault(), tile_repeat_x, tile_repeat_y); print(f"Applied texture '{texture_path}' with tiling {tile_repeat_x}x{tile_repeat_y} (1 tile per unit)")
        except Exception as e: print(f"ERROR loading/applying ground texture: {e}."); self.ground_visual_np.setColor(0.5, 0.5, 0.5, 1)
        debug_node = panda3d.bullet.BulletDebugNode('Debug'); debug_np = self.render.attachNewNode(debug_node); debug_np.show(); self.physics_world.setDebugNode(debug_node)

    def setup_barrier_sphere(self, radius):
        # Sets up central barrier object (stacked spheres).
        # (Includes collision masks + tag)
        print(f"Setting up 3 stacked spheres: Radius={radius}"); diameter = radius * 2.0; body_node = panda3d.bullet.BulletRigidBodyNode('StackedSphereBarrier'); body_node.setMass(0)
        body_np = self.render.attachNewNode(body_node); body_np.setPos(0, 0, 0); body_np.node().setIntoCollideMask(COLLISION_MASK_SPHERE); body_np.setCollideMask(COLLISION_MASK_PLAYER | COLLISION_MASK_ATTACKER | COLLISION_MASK_PROJECTILE); body_np.setPythonTag("type", "sphere"); body_np.setPythonTag("object", True)
        vis_sphere_model = self.loader.loadModel("models/misc/sphere")
        for i in range(3): z_pos = (i - 1) * diameter; heading = i * 120.0; transform = TransformState.makePosHpr(Point3(0, 0, z_pos), Vec3(heading, 0, 0)); shape = panda3d.bullet.BulletSphereShape(radius); body_node.addShape(shape, transform); vis_np = body_np.attachNewNode(f"vis_sphere_{i}"); vis_sphere_model.instanceTo(vis_np); vis_np.setTransform(transform); vis_np.setScale(radius); vis_np.setColor(1, 0, 0, 1)
        self.physics_world.attachRigidBody(body_node); print(f"Stacked Sphere Barrier created with collision masks.")
        return body_node, body_np

    def setup_player(self, radius, height):
        # Sets up the player object (physics).
        # (Includes collision masks + tag)
        cylinder_height = height - 2 * radius; shape = panda3d.bullet.BulletCapsuleShape(radius, cylinder_height, panda3d.bullet.Z_up)
        body_node = panda3d.bullet.BulletRigidBodyNode('Player'); body_node.addShape(shape); body_node.setMass(1.0); body_node.setLinearSleepThreshold(0); body_node.setAngularFactor(Vec3(0, 0, 0))
        halfway_radius = (self.sphere_radius + self.wall_radius) / 2.0; start_z = height * 0.5 + 0.1; start_pos = Point3(halfway_radius, 0.0, start_z)
        body_np = self.render.attachNewNode(body_node); body_np.setPos(start_pos); body_np.lookAt(Point3(0, 0, start_z))
        body_np.node().setIntoCollideMask(COLLISION_MASK_PLAYER); body_np.setCollideMask( COLLISION_MASK_GROUND | COLLISION_MASK_WALL | COLLISION_MASK_ATTACKER | COLLISION_MASK_SPHERE ); body_np.setPythonTag("type", "player"); body_np.setPythonTag("object", True)
        self.physics_world.attachRigidBody(body_node); print(f"Player object created: Mass=1.0, Position={start_pos}, Facing Origin, Dynamic")
        return body_node, body_np

    def setup_input(self):
        # Sets up keyboard/mouse controls and ESC exit.
        self.accept("w",self.set_key,["forward",True]);self.accept("w-up",self.set_key,["forward",False]); self.accept("s",self.set_key,["backward",True]);self.accept("s-up",self.set_key,["backward",False])
        self.accept("a",self.set_key,["left",True]);self.accept("a-up",self.set_key,["left",False]); self.accept("d",self.set_key,["right",True]);self.accept("d-up",self.set_key,["right",False])
        self.accept("space",self.set_key,["jump",True]);self.accept("space-up",self.set_key,["jump",False])
        self.accept("mouse1", self.trigger_attack); self.accept("mouse3", self.trigger_spit_attack)
        self.accept("escape", sys.exit) # Exit on ESC
        props = WindowProperties(); props.setCursorHidden(True); props.setMouseMode(WindowProperties.M_relative); self.win.requestProperties(props)

    def setup_blades(self):
        # Sets up the visual 'blades' attached to the camera.
        blade_model=self.loader.loadModel("models/misc/xyzAxis");blade_scale=Vec3(0.05,0.4,0.1);y_pos=0.6;z_pos=-0.2;x_offset=0.3;self.left_blade_np=NodePath("blade_left");blade_model.instanceTo(self.left_blade_np);self.left_blade_np.reparentTo(self.camera);self.left_blade_np.setScale(blade_scale);self.left_blade_start_pos=Point3(-x_offset,y_pos,z_pos);self.left_blade_np.setPos(self.left_blade_start_pos);self.left_blade_np.setHpr(0,-10,-15);self.left_blade_np.setColor(0.8,0.8,0.9,1);self.right_blade_np=NodePath("blade_right");blade_model.instanceTo(self.right_blade_np);self.right_blade_np.reparentTo(self.camera);self.right_blade_np.setScale(blade_scale);self.right_blade_np.setPos(x_offset,y_pos,z_pos);self.right_blade_np.setHpr(0,-10,15);self.right_blade_np.setColor(0.8,0.8,0.9,1);print("Blade representation created.")

    def setup_polygon_wall(self, num_sides, radius):
        # Sets up the polygon wall environment boundary.
        # (Includes collision masks + tag)
        wall_height=self.player_height*3.0;wall_thickness=0.5;texture_path="wall-blob.jpg";print(f"Setting up {num_sides}-sided polygon wall: Radius={radius}, Height={wall_height}")
        try: tex=self.loader.loadTexture(Filename.from_os_specific(texture_path));tex.setWrapU(Texture.WM_repeat);tex.setWrapV(Texture.WM_repeat);print(f"Loaded wall texture: {texture_path}")
        except Exception as e: print(f"ERROR loading wall texture: {e}.");tex=None
        wall_segment_model=self.loader.loadModel("models/box");vertices=[];angle_step=2.0*math.pi/num_sides
        for i in range(num_sides): angle=i*angle_step;vx=radius*math.cos(angle);vy=radius*math.sin(angle);vertices.append(Point3(vx,vy,0))
        wall_parent_np=self.render.attachNewNode("PolygonWall")
        for i in range(num_sides):
            p1=vertices[i];p2=vertices[(i+1)%num_sides];segment_vector=p2-p1;segment_length=segment_vector.length();center_pos=p1+segment_vector/2.0;center_pos.z=wall_height/2.0;angle_rad=math.atan2(segment_vector.y,segment_vector.x);angle_deg=math.degrees(angle_rad)
            shape_half_extents=Vec3(segment_length/2.0,wall_thickness/2.0,wall_height/2.0);wall_shape=panda3d.bullet.BulletBoxShape(shape_half_extents)
            wall_node=panda3d.bullet.BulletRigidBodyNode(f'PolyWall_{i}');wall_node.addShape(wall_shape);wall_node.setMass(0)
            wall_phys_np=wall_parent_np.attachNewNode(wall_node);wall_phys_np.setPos(center_pos);wall_phys_np.setH(angle_deg)
            wall_phys_np.node().setIntoCollideMask(COLLISION_MASK_WALL); wall_phys_np.setCollideMask(COLLISION_MASK_PLAYER | COLLISION_MASK_PROJECTILE); wall_phys_np.setPythonTag("type", "wall"); wall_phys_np.setPythonTag("object", True)
            self.physics_world.attachRigidBody(wall_node)
            vis_wall=NodePath(f"vis_polywall_{i}");wall_segment_model.instanceTo(vis_wall);vis_wall.reparentTo(wall_phys_np);vis_wall.setScale(segment_length/2.0,wall_thickness/2.0,wall_height/2.0)
            if tex: vis_wall.setTexture(tex);vis_wall.setTexScale(TextureStage.getDefault(),segment_length/4.0,wall_height/2.0)
            else: vis_wall.setColor(0.5,0.55,0.6,1)
        print(f"Created {num_sides} polygon wall segments with collision masks.")

    # Minimap removed

    def set_key(self, key, value): self.key_map[key] = value
    def update_physics(self, task): dt=globalClock.getDt(); self.physics_world.doPhysics(dt, 5, 1.0/180.0); return task.cont

    def update_player(self, task):
        # Runs every frame to update player movement based on key map.
        # Includes game_over check
        if self.game_over: return task.cont
        player_pos=self.player_np.getPos();move_speed=15.0;jump_force=8.0;current_vel=self.player_phys_node.getLinearVelocity();player_quat=self.player_np.getQuat();on_ground=False
        if self.player_phys_node.getNumShapes()>0:
            shape_actual=self.player_phys_node.getShape(0)
            if isinstance(shape_actual,panda3d.bullet.BulletCapsuleShape):
                shape_half_height=shape_actual.getHalfHeight();shape_radius=shape_actual.getRadius();ray_start_offset=-0.1;ray_length=abs(ray_start_offset)+shape_half_height+shape_radius+0.1;ray_start=player_pos+Vec3(0,0,ray_start_offset);ray_end=player_pos+Vec3(0,0,-ray_length);result=self.physics_world.rayTestClosest(ray_start,ray_end);on_ground=result.hasHit()
        player_forward=player_quat.getForward();player_right=player_quat.getRight();player_forward.z=0;player_right.z=0
        if player_forward.lengthSquared()>1e-6:player_forward.normalize()
        if player_right.lengthSquared()>1e-6:player_right.normalize()
        move_direction=Vec3(0,0,0)
        if self.key_map["forward"]:move_direction+=player_forward
        if self.key_map["backward"]:move_direction-=player_forward
        if self.key_map["left"]:move_direction-=player_right
        if self.key_map["right"]:move_direction+=player_right
        if move_direction.lengthSquared()>0:
            move_direction.normalize();target_vel_xy=move_direction*move_speed;self.player_phys_node.setLinearVelocity(Vec3(target_vel_xy.x,target_vel_xy.y,current_vel.z))
        else:self.player_phys_node.setLinearVelocity(Vec3(0,0,current_vel.z))
        if self.key_map["jump"]and on_ground:self.player_phys_node.applyCentralImpulse(Vec3(0,0,jump_force));self.key_map["jump"]=False
        if not self.player_phys_node.isActive()and(move_direction.lengthSquared()>0 or self.key_map["jump"]):self.player_phys_node.setActive(True,True)
        return task.cont

    def mouse_look_task(self, task):
        # Runs every frame to handle mouse look (rotation).
        # Includes game_over check and smoothing/deadzone
        if self.game_over: return task.cont
        mw=self.mouseWatcherNode;
        if mw.hasMouse():
            dx=mw.getMouseX();dy=mw.getMouseY();sensitivity_h=8.0;sensitivity_v=8.0;smoothing_factor=5.0;deadzone=0.5
            if abs(dx)>deadzone or abs(dy)>deadzone:
                dt=globalClock.getDt();current_h=self.player_np.getH();target_h=current_h-dx*sensitivity_h;diff_h=target_h-current_h;diff_h=(diff_h+180)%360-180;interp_t=min(smoothing_factor*dt,1.0);smoothed_h=current_h+diff_h*interp_t;self.player_np.setH(smoothed_h)
                current_p=self.camera.getP();target_p=current_p+dy*sensitivity_v;target_p=max(min(target_p,89.0),-89.0);diff_p=target_p-current_p;smoothed_p=current_p+diff_p*interp_t;self.camera.setP(smoothed_p)
        return task.cont

    def trigger_attack(self):
        # Triggers the melee attack animation.
        # Includes game_over check
        if self.game_over: return
        if not self.is_attacking:
            self.is_attacking = True; attack_up_pos = self.left_blade_start_pos + Point3(0, 0.1, 0.15); duration_up = 0.15; duration_down = 0.2
            move_up = LerpPosInterval(self.left_blade_np, duration_up, pos=attack_up_pos, startPos=self.left_blade_start_pos, blendType='easeOut')
            check_hit = Func(self.perform_attack_check); move_down = LerpPosInterval(self.left_blade_np, duration_down, pos=self.left_blade_start_pos, startPos=attack_up_pos, blendType='easeIn')
            finish_attack = Func(setattr, self, 'is_attacking', False); self.attack_sequence = Sequence(move_up, check_hit, move_down, finish_attack); self.attack_sequence.start()

    def perform_attack_check(self):
        # Performs hit detection for melee attack using simple distance check.
        # TODO Refine Melee hit detection (e.g., shape test)
        print("Attack Hit Frame! Check for ATTACKERS here (Distance Check)."); attack_check_dist = 2.0; hit_radius_sq = 1.0 * 1.0
        player_forward = self.player_np.getQuat(self.render).getForward(); player_center_pos = self.player_np.getPos(self.render)
        attack_center_pos = player_center_pos + player_forward * attack_check_dist; print(f"  Checking for hit near {attack_center_pos}")
        for attacker in self.attackers_list[:]:
            if attacker.nodepath and not attacker.nodepath.isEmpty():
                attacker_pos = attacker.nodepath.getPos(self.render); dist_sq = (attacker_pos - attack_center_pos).lengthSquared()
                if dist_sq < hit_radius_sq: print(f"  >>> HIT (Distance Check) ATTACKER: {attacker.body_node.getName()}!"); attacker.despawn()

    def trigger_spit_attack(self):
        # Creates and launches a projectile.
        # Includes game_over check
        if self.game_over: return
        print("Spit Attack Triggered!"); cam_forward = self.camera.getQuat(self.render).getForward()
        start_pos = self.camera.getPos(self.render) + cam_forward * 0.5; direction = cam_forward
        projectile = Projectile(self, start_pos, direction)

    def spawn_attacker(self, task=None): # Added task=None argument
        # Creates a new attacker instance on the ground 1-3 units INSIDE the wall.
        spawn_offset = random.uniform(1.0, 3.0); spawn_radius = self.wall_radius - spawn_offset
        angle = random.uniform(0, 2.0 * math.pi); spawn_x = spawn_radius * math.cos(angle); spawn_y = spawn_radius * math.sin(angle)
        spawn_z = ATTACKERS.physics_radius # Use physics radius
        start_pos = Point3(spawn_x, spawn_y, spawn_z)
        # print(f"Attempting to spawn attacker at {start_pos} (Offset from wall: {spawn_offset:.2f})") # Reduce logging
        attacker = ATTACKERS(self, start_pos); self.attackers_list.append(attacker)
        return Task.done # Use Task.done

    def spawn_two_attackers(self, task=None):
        # Helper method to spawn two attackers, called by doMethodLater.
        # print("Spawning two attackers...") # Reduce logging
        self.spawn_attacker()
        self.spawn_attacker()
        return Task.done

    def update_attackers(self, task):
        # Task to update all active attackers.
        # Includes game_over check
        if self.game_over: return task.cont
        dt = globalClock.getDt()
        for attacker in self.attackers_list[:]: # Iterate copy
            if attacker.nodepath and not attacker.nodepath.isEmpty(): attacker.update(dt) # Call attacker's update
            else: # Clean up list if attacker was already removed
                 if attacker in self.attackers_list: self.attackers_list.remove(attacker)
        return task.cont

    def on_bullet_contact(self, node1, node2):
        # Handles collision events, specifically for projectiles.
        # TODO Refine Projectile hit effects (damage, visuals, sounds)
        if self.game_over: return # Ignore contacts if game over

        # Use hasPythonTag for safety before getting tags
        type1 = node1.getPythonTag("type") if node1.hasPythonTag("type") else None
        type2 = node2.getPythonTag("type") if node2.hasPythonTag("type") else None
        obj1 = node1.getPythonTag("object") if node1.hasPythonTag("object") else None
        obj2 = node2.getPythonTag("object") if node2.hasPythonTag("object") else None

        projectile = None; other_node = None; other_type = None; other_obj = None

        # Identify projectile and the other object/type
        if type1 == "projectile" and obj1: projectile = obj1; other_node = node2; other_type = type2; other_obj = obj2
        elif type2 == "projectile" and obj2: projectile = obj2; other_node = node1; other_type = type1; other_obj = obj1

        if projectile and projectile.nodepath: # Check projectile still exists
            # print(f"Projectile contact with type: {other_type}") # Reduce logging
            if other_type == "attacker":
                if other_obj and other_obj.nodepath: # Check attacker still exists
                    print(f"  Projectile hit ATTACKER: {other_obj.body_node.getName()}")
                    other_obj.despawn() # Despawn the attacker
                    projectile.despawn() # Despawn the projectile
            elif other_type in ["wall", "ground", "sphere"]:
                # print(f"  Projectile hit environment ({other_type})") # Reduce logging
                projectile.despawn() # Despawn projectile only

    # --- Failure Condition Handling ---
    def attacker_reached_center(self):
        # Called by an attacker when it contacts the central sphere.
        if self.game_over: return
        current_time = globalClock.getFrameTime(); self.recent_center_touches.append(current_time)
        print(f"Center touched! Queue size: {len(self.recent_center_touches)}")
        # Prune timestamps older than the time window
        while self.recent_center_touches and (current_time - self.recent_center_touches[0] > self.failure_time_window): self.recent_center_touches.popleft()
        # Check if failure condition met
        if len(self.recent_center_touches) >= self.failure_touch_count: self.handle_failure()

    def handle_failure(self):
        # Triggers the game over sequence.
        if self.game_over: return
        print("!!! FAILURE CONDITION MET !!! 10 touches in 3 seconds."); self.game_over = True
        # Stop further center touch checks immediately by setting flag
        self.taskMgr.doMethodLater(0.1, self.flash_white_and_reset, 'FlashResetTask')

    def flash_white_and_reset(self, task=None):
        # Flashes screen white, then schedules game reset.
        print("Flashing white...");
        if not hasattr(self, 'original_bg_color'): self.original_bg_color = self.getBackgroundColor()
        self.setBackgroundColor(1, 1, 1, 1); self.taskMgr.doMethodLater(0.5, self.reset_game, 'ResetGameTask')
        return Task.done

    def reset_game(self, task=None):
        # Resets the game state to initial values after failure.
        # TODO Implement Retry Button via DirectGUI instead of auto-reset
        print("Resetting game..."); self.setBackgroundColor(self.original_bg_color); self.player_health = 3; print(f"Player health reset to: {self.player_health}")
        halfway_radius = (self.sphere_radius + self.wall_radius) / 2.0; start_z = self.player_height * 0.5 + 0.1; start_pos = Point3(halfway_radius, 0.0, start_z)
        self.player_phys_node.setLinearVelocity(Vec3(0,0,0)); self.player_phys_node.setAngularVelocity(Vec3(0,0,0))
        self.player_np.setPosHpr(start_pos, Point3(0,0,0)); self.player_np.lookAt(Point3(0, 0, start_z)); self.player_phys_node.setActive(True, True)
        # Remove pending respawn tasks
        print("Removing pending respawn tasks...")
        removed_count = self.taskMgr.removeTasksMatching("RespawnAttackerTask_*")
        print(f"  Removed {removed_count} pending respawn tasks.")
        # Clear existing attackers
        print("Clearing existing attackers...")
        for attacker in self.attackers_list[:]:
             if attacker.nodepath and not attacker.nodepath.isEmpty():
                  self.physics_world.removeRigidBody(attacker.body_node); attacker.nodepath.removeNode()
        self.attackers_list.clear(); self.recent_center_touches.clear()
        print("Spawning initial attacker...")
        self.spawn_attacker() # Spawn the single starting attacker
        self.game_over = False; print("Game Reset Complete.")
        return Task.done
    # ---------------------------------------

# --- Run the Application ---
if __name__ == "__main__":
    app = PhysicsWorldApp()
    app.run() # Runs windowed at 1366x768 based on config
