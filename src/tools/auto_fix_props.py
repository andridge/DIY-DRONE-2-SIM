# ...existing code...
"""
Auto-fix propeller clearance:
 - Measures mesh radii for a_1, a`_1, b_1, b`_1
 - Computes top-pair clearance (target)
 - Nudges or scales bottom pair to match target clearance
 - Edits visual and collision origin xyz and mesh scale in august.xacro
 - Makes a backup august.xacro.bak
Usage:
  python auto_fix_props.py
"""
import os
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

# Use the user's catkin workspace august_description path
REPO_ROOT = os.path.expanduser('~/catkin_ws/src/DIY-DRONE-2-SIM/src/august_description')
XACRO_PATH = os.path.join(REPO_ROOT, 'urdf', 'august.xacro')
LINKS = ['a_1', 'a`_1', 'b_1', 'b`_1']

def parse_xyz(attr):
    if not attr:
        return np.zeros(3)
    parts = [float(x) for x in attr.strip().split()]
    return np.array(parts)

def mesh_path(filename):
    if not filename:
        return None
    if filename.startswith('package://'):
        # package://<pkg>/<path...> -> find path relative to REPO_ROOT
        rel = filename.split('package://', 1)[1]
        parts = rel.split('/', 1)
        # drop leading package name if present
        rel_path = parts[1] if len(parts) > 1 else parts[0]
        candidate = os.path.join(REPO_ROOT, rel_path.replace('/', os.sep))
        return os.path.normpath(candidate)
    return os.path.normpath(os.path.expanduser(filename))

def mesh_radius(meshfile, scale=(1.0,1.0,1.0)):
    m = trimesh.load(meshfile, force='mesh')
    verts = (np.asarray(m.vertices) * np.array(scale))
    return float(np.max(np.linalg.norm(verts, axis=1)))

def find_visual_collision(link_elem):
    vis = link_elem.find('visual')
    col = link_elem.find('collision')
    return vis, col

def get_link_info(root, name):
    for link in root.findall('.//link'):
        if link.get('name') == name:
            vis, col = find_visual_collision(link)
            info = {'elem': link, 'visual': vis, 'collision': col}
            # visual origin and mesh
            if vis is not None:
                orig = vis.find('origin')
                geom = vis.find('geometry')
                mesh = None; scale = (1.0,1.0,1.0)
                if geom is not None:
                    mesh_elem = geom.find('.//mesh')
                    if mesh_elem is not None:
                        mesh = mesh_elem.get('filename')
                        s = mesh_elem.get('scale')
                        if s:
                            scale = tuple(float(x) for x in s.split())
                info['vis_origin'] = parse_xyz(orig.get('xyz') if orig is not None else None)
                info['mesh'] = mesh
                info['scale'] = scale
            # collision origin & mesh
            if col is not None:
                orig = col.find('origin')
                geom = col.find('geometry')
                mesh = None; scale = (1.0,1.0,1.0)
                if geom is not None:
                    mesh_elem = geom.find('.//mesh')
                    if mesh_elem is not None:
                        mesh = mesh_elem.get('filename')
                        s = mesh_elem.get('scale')
                        if s:
                            scale = tuple(float(x) for x in s.split())
                info['col_origin'] = parse_xyz(orig.get('xyz') if orig is not None else None)
                info['col_mesh'] = mesh
                info['col_scale'] = scale
            return info
    return None

def set_origin(elem, tag, xyz):
    node = elem.find(tag)
    if node is None:
        node = ET.SubElement(elem, tag)
    node.set('xyz', f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}")

def set_mesh_scale(geom_elem, scale_tuple):
    mesh_elem = geom_elem.find('.//mesh')
    if mesh_elem is None:
        return
    mesh_elem.set('scale', f"{scale_tuple[0]:.6f} {scale_tuple[1]:.6f} {scale_tuple[2]:.6f}")

def mm(x): return x*1000.0

def main():
    if not os.path.exists(XACRO_PATH):
        print("Xacro not found:", XACRO_PATH)
        return

    tree = ET.parse(XACRO_PATH)
    root = tree.getroot()

    infos = {}
    missing_meshes = False
    for ln in LINKS:
        info = get_link_info(root, ln)
        if not info:
            print("Link not found in xacro:", ln)
            return
        infos[ln] = info
        # compute radii
        meshfile = mesh_path(info.get('mesh'))
        if meshfile and os.path.exists(meshfile):
            try:
                infos[ln]['radius'] = mesh_radius(meshfile, info.get('scale'))
            except Exception as e:
                print(f"ERROR loading mesh for {ln}: {meshfile} -> {e}")
                infos[ln]['radius'] = None
                missing_meshes = True
        else:
            infos[ln]['radius'] = None
            missing_meshes = True
            print(f"WARNING: mesh for link {ln} not found at: {meshfile}")

    if missing_meshes:
        print("Missing mesh data; cannot compute. Ensure meshes exist under:")
        print("  ", REPO_ROOT)
        return

    # compute center distances and clearances
    def pair_stats(l1, l2):
        p1 = infos[l1]['vis_origin']
        p2 = infos[l2]['vis_origin']
        dist = float(np.linalg.norm(p1 - p2))
        r1 = infos[l1]['radius']; r2 = infos[l2]['radius']
        sumr = r1 + r2
        clearance = sumr - dist
        return {'dist': dist, 'sumr': sumr, 'clearance': clearance}

    top = pair_stats('a_1','a`_1')
    bot = pair_stats('b_1','b`_1')

    print(f"Top pair center dist {mm(top['dist']):.1f} mm, sum radii {mm(top['sumr']):.1f} mm, clearance {mm(top['clearance']):.1f} mm")
    print(f"Bottom pair center dist {mm(bot['dist']):.1f} mm, sum radii {mm(bot['sumr']):.1f} mm, clearance {mm(bot['clearance']):.1f} mm")

    target_clearance = top['clearance']
    current_clearance = bot['clearance']
    diff = target_clearance - current_clearance  # positive => bottom is tighter (more overlap) than top

    if abs(diff) < 1e-4:
        print("Bottom pair already matches top pair clearance. No change required.")
        return

    # derive required center distance increase
    dist_new = bot['sumr'] - target_clearance
    increase = dist_new - bot['dist']
    print(f"Computed required center distance increase: {mm(increase):.2f} mm")

    max_nudge = 0.02  # 20 mm max per prop
    delta = increase / 2.0

    # direction between current centers (pointing b_1 - b`_1)
    p1 = infos['b_1']['vis_origin']; p2 = infos['b`_1']['vis_origin']
    vec = p1 - p2
    if np.linalg.norm(vec) < 1e-6:
        dirx = np.array([1.0,0.0,0.0])
    else:
        dirx = vec / np.linalg.norm(vec)

    do_scale = False
    if abs(delta) > max_nudge:
        s = (bot['dist'] + target_clearance) / bot['sumr']
        s = max(0.75, min(1.0, s))
        print(f"Nudge too large ({mm(delta):.1f} mm). Will scale bottom props by factor {s:.4f} instead.")
        do_scale = True
    else:
        print(f"Applying nudge of {mm(delta):.2f} mm to each bottom prop along vector {dirx}.")

    # backup original
    bak = XACRO_PATH + '.bak'
    shutil.copy2(XACRO_PATH, bak)
    print("Backup created:", bak)

    if do_scale:
        scale_factor = s
        for name in ('b_1','b`_1'):
            info = infos[name]
            vis = info['visual']
            if vis is not None:
                geom = vis.find('geometry')
                if geom is not None:
                    set_mesh_scale(geom, (scale_factor, scale_factor, scale_factor))
            col = info.get('collision')
            if col is not None:
                geomc = col.find('geometry')
                if geomc is not None:
                    set_mesh_scale(geomc, (scale_factor, scale_factor, scale_factor))
    else:
        move = dirx * delta
        b1_new = infos['b_1']['vis_origin'] + move
        bn_new = infos['b`_1']['vis_origin'] - move
        for name, newpos in [('b_1', b1_new), ('b`_1', bn_new)]:
            info = infos[name]
            vis = info['visual']
            col = info.get('collision')
            if vis is not None:
                origin_vis = vis.find('origin')
                if origin_vis is None:
                    origin_vis = ET.SubElement(vis, 'origin')
                origin_vis.set('xyz', f"{newpos[0]:.6f} {newpos[1]:.6f} {newpos[2]:.6f}")
            if col is not None:
                origin_col = col.find('origin')
                if origin_col is None:
                    origin_col = ET.SubElement(col, 'origin')
                origin_col.set('xyz', f"{newpos[0]:.6f} {newpos[1]:.6f} {newpos[2]:.6f}")

    tree.write(XACRO_PATH, encoding='utf-8', xml_declaration=True)
    print("Updated xacro written to:", XACRO_PATH)
    print("Restart Gazebo and test. Re-run script to iterate if needed.")

if __name__ =='__main__':
    main()