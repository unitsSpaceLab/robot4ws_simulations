import tempfile
import subprocess
import os
from lxml import etree


def generate_temp_world_with_xacro(
    base_world_path,
    urdf_xml,
    model_name="my_rover",
    pose="0 0 0 0 0 0",
):
    """
    Creates a temporary world file containing a model generated from xacro.

    Returns:
        temp_world_path (str)
        cleanup_fn (callable)  -> call this on shutdown to delete generated world
    """

    # dump urdf in tmp file 
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp_urdf:
        tmp_urdf.write(urdf_xml.encode("utf-8"))
        tmp_urdf_path = tmp_urdf.name

    # -------------------------
    # Convert URDF -> SDF using gz
    # -------------------------
    sdf_xml = subprocess.check_output(
        ["gz", "sdf", "-p", tmp_urdf_path],
        text=True
    )

    # remove tmp urdf file
    os.remove(tmp_urdf_path)

    sdf_root = etree.fromstring(sdf_xml.encode())

    # gz sdf -p may output <sdf><model>...</model></sdf>
    model_elem = sdf_root.find("model")
    if model_elem is None:
        raise RuntimeError("No <model> in generated SDF")

    model_elem.attrib["name"] = model_name

    # set pose if provided
    pose_elem = etree.SubElement(model_elem, "pose")
    pose_elem.text = pose

    # -------------------------
    # Load base world
    # -------------------------
    tree = etree.parse(base_world_path)
    root = tree.getroot()

    # world element
    world_elem = root.find("world")
    if world_elem is None:
        raise RuntimeError("No <world> in base world")

    # -------------------------
    # Inject model into world
    # -------------------------
    world_elem.append(model_elem)

    # -------------------------
    # Write temporary world
    # -------------------------
    tmp = tempfile.NamedTemporaryFile(
        suffix=".world",
        delete=False
    )
    tree.write(tmp.name, pretty_print=True)

    temp_path = tmp.name
    tmp.close()

    # -------------------------
    # Cleanup function
    # -------------------------
    def cleanup():
        if os.path.exists(temp_path):
            os.remove(temp_path)
            print(f"Temporary generated world [{temp_path}] removed")

    return temp_path, cleanup


def find_world_file(world_name):
    search_paths = os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep)
    for path in search_paths:
        candidate = os.path.join(path, world_name)
        # print(candidate)
        if os.path.isfile(candidate):
            return candidate

    # default gz worlds location
    candidate = os.path.join("/usr/share/gz/gz-sim8/worlds", world_name)
    if os.path.isfile(candidate):
        return candidate

    return ""
