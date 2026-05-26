from pathlib import Path

PYBULLET_DIR = Path(__file__).resolve().parent

original_urdf_path = (
    PYBULLET_DIR
    / "modules"
    / "urdf"
    / "Assem1.SLDASM"
    / "urdf"
    / "Assem1.SLDASM.urdf"
)

mesh_dir = (
    PYBULLET_DIR
    / "modules"
    / "urdf"
    / "Assem1.SLDASM"
    / "meshes"
)

def make_fixed_urdf():
    urdf_content = original_urdf_path.read_text()

    mesh_dir_fixed = mesh_dir.resolve().as_posix() + "/"

    replacements = {
        "C:/Users/user/Desktop/modular_mobile_robots/modules/urdf/Assem1.SLDASM/meshes/": mesh_dir_fixed,
        "C:\\Users\\user\\Desktop\\modular_mobile_robots\\modules\\urdf\\Assem1.SLDASM\\meshes\\": mesh_dir_fixed,
        ":package//Assem1.SLDASM/meshes/": mesh_dir_fixed,
        "package://Assem1.SLDASM/meshes/": mesh_dir_fixed,
    }

    for old, new in replacements.items():
        urdf_content = urdf_content.replace(old, new)

    fixed_urdf_path = original_urdf_path.parent / "Assem1_fixed_pybullet.urdf"
    fixed_urdf_path.write_text(urdf_content)

    return fixed_urdf_path.resolve().as_posix()

ROBOT_URDF_PATH = make_fixed_urdf()

configurations = ["line", "arrow"]
width = 0.0749
circumcircle_radius = 0.08649
timestep = 1. / 240.ROBOT_URDF_PATH = "C:/Users/user/Desktop/modular_mobile_robots/modules/urdf/Assem1.SLDASM/urdf/Assem1.SLDASM.urdf"
configurations = ["line", "arrow"]  # List of available configurations
width = 0.0749  # Width of the module 
circumcircle_radius = 0.08649  # Circumcircle radius for a regular hexagon
timestep = 1. / 240.
