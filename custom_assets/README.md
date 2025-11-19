# Custom Assets for Isaac Sim - Official Workflow

## Directory Structure

Place your source assets in the following directories:

- **robots/** - URDF and MJCF robot models (will be converted to USD)
- **environments/** - OBJ, FBX, STL, glTF environment models (will be converted to USD)
- **textures/** - Texture files (PNG, JPG, HDR)
- **materials/** - Material definitions (MDL files)
- **configs/** - Configuration files

## Setup Complete

✅ Directory structure created
✅ Docker volume mount configured (`./custom_assets:/workspace/custom_assets`)
✅ Using official Isaac Lab conversion tools

## Official Asset Import Workflow

### 1. Start Docker Containers
```bash
cd isaac-launchable/isaac-lab
docker compose up -d
```

### 2. Access VS Code Development Environment
- Open browser to: `http://localhost`
- Password: `password`

### 3. Convert Assets Using Official Isaac Lab Tools

The Isaac Lab container includes official conversion scripts at:
- `scripts/tools/convert_urdf.py` - For robot URDF files
- `scripts/tools/convert_mjcf.py` - For MuJoCo models
- `scripts/tools/convert_mesh.py` - For mesh files (OBJ, FBX, STL, glTF)

#### Convert URDF Robot to USD

```bash
# Basic conversion
./isaaclab.sh -p scripts/tools/convert_urdf.py \
  /workspace/custom_assets/robots/my_robot.urdf \
  /workspace/custom_assets/robots/my_robot.usd

# With options for better performance
./isaaclab.sh -p scripts/tools/convert_urdf.py \
  /workspace/custom_assets/robots/my_robot.urdf \
  /workspace/custom_assets/robots/my_robot.usd \
  --make-instanceable \
  --merge-fixed-joints \
  --fix-base
```

**Common URDF Conversion Options:**
- `--fix-base` - Fix the robot base (for manipulators)
- `--make-instanceable` - Optimize for multiple instances (important for large-scale simulation)
- `--merge-fixed-joints` - Merge fixed joints to reduce complexity
- `--self-collision` - Enable self-collision checking
- `--convex-decomposition` - Decompose meshes for better collision detection

#### Convert Environment Meshes to USD

```bash
# Convert OBJ/FBX/STL to USD
./isaaclab.sh -p scripts/tools/convert_mesh.py \
  /workspace/custom_assets/environments/warehouse.obj \
  /workspace/custom_assets/environments/warehouse.usd \
  --mass 1000.0 \
  --collision-approximation convexDecomposition
```

#### Convert MuJoCo Models

```bash
./isaaclab.sh -p scripts/tools/convert_mjcf.py \
  /workspace/custom_assets/robots/my_robot.xml \
  /workspace/custom_assets/robots/my_robot.usd \
  --make-instanceable
```

### 4. Launch Isaac Sim with Your Converted Assets

#### Option A: Basic Isaac Sim with GUI
```bash
./isaaclab/_isaac_sim/isaac-sim.sh --no-window --enable omni.kit.livestream.webrtc
```
Then manually load your USD files through the Isaac Sim interface.

#### Option B: Use in Isaac Lab Scripts

Create or modify Isaac Lab scripts to use your assets:

```python
# Example: In any Isaac Lab tutorial or custom script
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World

# Initialize world
world = World()

# Load your converted robot
add_reference_to_stage(
    usd_path="/workspace/custom_assets/robots/my_robot.usd",
    prim_path="/World/Robot"
)

# Load your converted environment
add_reference_to_stage(
    usd_path="/workspace/custom_assets/environments/warehouse.usd",
    prim_path="/World/Environment"
)

world.reset()
```

Run your script:
```bash
./isaaclab.sh -p your_script.py --kit_args="--no-window --enable omni.kit.livestream.webrtc"
```

### 5. View the Simulation
- Open new browser tab: `http://localhost/viewer`
- Wait for streaming connection (first launch may take longer due to shader compilation)

## Using the Python API Directly

For programmatic URDF import without pre-conversion:

```python
from isaacsim.asset.importer.urdf import _urdf
import omni.kit.commands

# Configure import
import_config = _urdf.ImportConfig()
import_config.fix_base = True
import_config.make_instanceable = True
import_config.merge_fixed_joints = True
import_config.convex_decomp = False
import_config.self_collision = False
import_config.density = 0.0  # Use default densities

# Parse URDF
result, robot_model = omni.kit.commands.execute(
    "URDFParseFile",
    urdf_path="/workspace/custom_assets/robots/my_robot.urdf",
    import_config=import_config
)

# Import to stage
result, prim_path = omni.kit.commands.execute(
    "URDFImportRobot",
    urdf_robot=robot_model,
    import_config=import_config,
    stage_path="/World/MyRobot"
)
```

## Important Notes

### Asset Performance
- **Always use `--make-instanceable`** for robots that will be spawned multiple times
- This significantly improves memory usage and performance in large-scale simulations

### File Formats
- **Input formats supported:**
  - Robots: URDF, MJCF
  - Meshes: OBJ, FBX, STL, glTF
- **Output format:** USD (Universal Scene Description)

### First Load Times
- Assets take longer to load the first time (multiple minutes for complex robots)
- Subsequent loads are much faster due to caching

### Container Paths
Inside the container, your assets are accessible at:
- Source files: `/workspace/custom_assets/`
- Converted USD files: Same directory as source (unless specified otherwise)

## Troubleshooting

### Check Container Status
```bash
docker ps
```

Should show:
- vscode
- web-viewer
- nginx

### View Container Logs
```bash
docker logs vscode
```

### Restart Containers
```bash
cd isaac-launchable/isaac-lab
docker compose down
docker compose up -d
```

### Common Issues
- **Import fails:** Check URDF is valid and all mesh files are referenced correctly
- **Assets not visible:** Ensure USD conversion completed successfully
- **Performance issues:** Use `--make-instanceable` flag during conversion

## Official Documentation

For more details, refer to:
- [Isaac Sim URDF Import Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/import_urdf.html)
- [Isaac Lab Asset Import Guide](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)