#!/usr/bin/python3
# by Joric, https://github.com/joric/io_scene_b3d

try:
    from B3DParser import *
except:
    pass

try:
    from .B3DParser import *
    import bpy
    import mathutils
    from bpy_extras.image_utils import load_image
    from bpy_extras.io_utils import unpack_list, unpack_face_list
    import bmesh
except:
    pass

def flip(v):
    return ((v[0],v[2],v[1]) if len(v)<4 else (v[0], v[1],v[3],v[2]))

def flip_all(v):
    return [y for y in [flip(x) for x in v]]

material_mapping = {}
weighting = {}
imported_armature_objects = []

def import_animations(data):
    frames = data.get('frames', 1)
    fps = data.get('fps', 30) or 30
    print(f"[b3d] animation: frames={frames}, fps={fps}, armatures={len(imported_armature_objects)}")
    if frames <= 1 or not imported_armature_objects:
        print("[b3d] no animation to import (no frames or no armature)")
        return

    def collect_key_nodes(node, out):
        if node.get('keys') and node.name:
            out[node.name] = node
            print(f"[b3d] found keys for node '{node.name}' ({len(node['keys'])} keys)")
        for child in node.get('nodes', []):
            collect_key_nodes(child, out)

    key_nodes = {}
    for root in data.get('nodes', []):
        collect_key_nodes(root, key_nodes)

    print(f"[b3d] key_nodes: {list(key_nodes.keys())}")
    if not key_nodes:
        print("[b3d] no nodes with keys found")
        return

    for arm_obj in imported_armature_objects:
        action = bpy.data.actions.new(name='b3d_action')
        action.use_fake_user = True
        arm_obj.animation_data_create()
        arm_obj.animation_data.action = action

        bone_names = [b.name for b in arm_obj.data.bones]
        print(f"[b3d] armature bones: {bone_names}")
        created_fcurves = []

        for bone in arm_obj.data.bones:
            node = key_nodes.get(bone.name)
            if not node:
                continue
            keys = node['keys']
            if not keys:
                continue

            rest_pos = flip(node.get('position', (0,0,0)))
            rest_rot = mathutils.Quaternion(flip(node.get('rotation', (1,0,0,0))))
            rest_scl = flip(node.get('scale', (1,1,1)))

            pos_keys = [k for k in keys if 'position' in k]
            rot_keys = [k for k in keys if 'rotation' in k]
            scl_keys = [k for k in keys if 'scale' in k]

            if pos_keys:
                first = pos_keys[0].frame
                n = len(pos_keys)
                print(f"[b3d] bone '{bone.name}': {n} position keys")
                for axis in range(3):
                    fc = action.fcurve_ensure_for_datablock(arm_obj, f'pose.bones["{bone.name}"].location', index=axis, group_name=bone.name)
                    fc.keyframe_points.add(n)
                    for i, k in enumerate(pos_keys):
                        val = flip(k.position)[axis] - rest_pos[axis]
                        fc.keyframe_points[i].co = (k.frame - first, val)
                        fc.keyframe_points[i].interpolation = 'LINEAR'
                    created_fcurves.append(fc)

            if rot_keys:
                first = rot_keys[0].frame
                n = len(rot_keys)
                print(f"[b3d] bone '{bone.name}': {n} rotation keys")
                for axis in range(4):
                    fc = action.fcurve_ensure_for_datablock(arm_obj, f'pose.bones["{bone.name}"].rotation_quaternion', index=axis, group_name=bone.name)
                    fc.keyframe_points.add(n)
                    for i, k in enumerate(rot_keys):
                        key_q = mathutils.Quaternion(flip(k.rotation))
                        relative_q = rest_rot.inverted() @ key_q
                        fc.keyframe_points[i].co = (k.frame - first, relative_q[axis])
                        fc.keyframe_points[i].interpolation = 'LINEAR'
                    created_fcurves.append(fc)

            if scl_keys:
                first = scl_keys[0].frame
                n = len(scl_keys)
                print(f"[b3d] bone '{bone.name}': {n} scale keys")
                for axis in range(3):
                    fc = action.fcurve_ensure_for_datablock(arm_obj, f'pose.bones["{bone.name}"].scale', index=axis, group_name=bone.name)
                    fc.keyframe_points.add(n)
                    for i, k in enumerate(scl_keys):
                        val = flip(k.scale)[axis] / rest_scl[axis] if rest_scl[axis] != 0 else 1.0
                        fc.keyframe_points[i].co = (k.frame - first, val)
                        fc.keyframe_points[i].interpolation = 'LINEAR'
                    created_fcurves.append(fc)

        print(f"[b3d] created {len(created_fcurves)} fcurves for action '{action.name}'")
        for fc in created_fcurves:
            fc.update()

def import_mesh(node, parent):
    global material_mapping

    mesh = bpy.data.meshes.new(node.name)

    faces = []
    for face in node.faces:
        faces.extend(face.indices)

    mesh.from_pydata(flip_all(node.vertices), [], flip_all(faces))

    mesh.vertices.foreach_set('normal', unpack_list(node.normals))

    ob = bpy.data.objects.new(node.name, mesh)

    bpymesh = ob.data
    uvs = [(0,0) if len(uv)==0 else (uv[0], 1-uv[1]) for uv in node.uvs]
    uvlist = [i for poly in bpymesh.polygons for vidx in poly.vertices for i in uvs[vidx]]
    bpymesh.uv_layers.new().data.foreach_set('uv', uvlist)

    for key, value in material_mapping.items():
        ob.data.materials.append(bpy.data.materials[value])

    poly = 0
    for face in node.faces:
        for _ in face.indices:
            ob.data.polygons[poly].material_index = face.brush_id
            poly += 1

    return ob

def select_recursive(root):
    for c in root.children:
        select_recursive(c)
    root.select_set(state=True)

def set_editbone_rotation(bone, desired_quat):
    bone.roll = 0.0
    q0 = bone.matrix.to_quaternion()
    diff = q0.inverted() @ desired_quat
    angle = diff.angle
    axis = diff.axis
    y_local = mathutils.Vector((0.0, 1.0, 0.0))
    if axis.dot(y_local) > 0.5:
        bone.roll = angle
    elif axis.dot(y_local) < -0.5:
        bone.roll = -angle
    else:
        bone.roll = 0.0

def make_armature_recursive(root, a, parent_bone):
    bone = a.data.edit_bones.new(root.name)
    world_mat = root.matrix_world
    head = world_mat.to_translation()
    desired_quat = world_mat.to_quaternion()
    y_axis = desired_quat @ mathutils.Vector((0.0, 1.0, 0.0))

    children = root.children
    if children:
        child_pos = children[0].matrix_world.to_translation()
        length = (child_pos - head).length
    else:
        length = 1.0
    if length < 0.01:
        length = 1.0

    bone.head = head
    bone.tail = head + y_axis * length
    bone.parent = parent_bone
    set_editbone_rotation(bone, desired_quat)

    for c in root.children:
        make_armature_recursive(c, a, bone)

def make_armatures():
    global ctx
    global imported_armatures, weighting, imported_armature_objects

    for dummy_root in imported_armatures:
        objName = 'armature'
        a = bpy.data.objects.new(objName, bpy.data.armatures.new(objName))
        imported_armature_objects.append(a)
        ctx.scene.collection.objects.link(a)
        for i in bpy.context.selected_objects: i.select_set(state=False)
        a.select_set(state=True)
        a.show_in_front = True
        a.data.display_type = 'OCTAHEDRAL'
        bpy.context.view_layer.objects.active = a

        bpy.ops.object.mode_set(mode='EDIT',toggle=False)
        make_armature_recursive(dummy_root, a, None)
        bpy.ops.object.mode_set(mode='OBJECT',toggle=False)

        ob = dummy_root.parent
        a.parent = ob

        for i in bpy.context.selected_objects: i.select_set(state=False)
        select_recursive(dummy_root)
        bpy.ops.object.delete(use_global=True)

        modifier = ob.modifiers.new(type="ARMATURE", name="armature")
        modifier.object = a

        for bone in a.data.bones.values():
            group = ob.vertex_groups.new(name=bone.name)
            if bone.name in weighting.keys():
                for vertex_id, weight in weighting[bone.name]:
                    group_indices = [vertex_id]
                    group.add(group_indices, weight, 'REPLACE')
        a.parent.data.update()

def import_bone(node, parent=None):
    global imported_armatures, weighting
    ob = bpy.data.objects.new(node.name, None)

    w = []
    for vert_id, weight in node['bones']:
        w.append((vert_id, weight))
    weighting[node.name] = w

    if parent and parent.type=='MESH':
        imported_armatures.append(ob)

    return ob

def import_node_recursive(node, parent=None):
    ob = None

    if 'vertices' in node and 'faces' in node:
        ob = import_mesh(node, parent)
    elif 'bones' in node:
        ob = import_bone(node, parent)
    elif node.name:
        ob = bpy.data.objects.new(node.name, None)

    if ob:
        ctx.scene.collection.objects.link(ob)

        if parent:
            ob.parent = parent

        ob.rotation_mode='QUATERNION'
        ob.rotation_quaternion = flip(node.rotation)
        ob.scale = flip(node.scale)
        ob.location = flip(node.position)

    for x in node.nodes:
        import_node_recursive(x, ob)

def load_b3d(filepath,
             context,
             IMPORT_CONSTRAIN_BOUNDS=10.0,
             IMAGE_SEARCH=True,
             APPLY_MATRIX=True,
             global_matrix=None):

    global ctx
    global material_mapping

    ctx = context
    data = B3DTree().parse(filepath)

    images = {}
    dirname = os.path.dirname(filepath)
    for i, texture in enumerate(data['textures'] if 'textures' in data else []):
        texture_name = os.path.basename(texture['name'])
        for mat in data.materials:
            if mat.tids[0]==i:
                images[i] = (texture_name, load_image(texture_name, dirname, check_existing=True,
                    place_holder=False, recursive=IMAGE_SEARCH))

    material_mapping = {}
    for i, mat in enumerate(data.materials if 'materials' in data else []):
        material = bpy.data.materials.new(mat.name)
        material_mapping[i] = material.name
        material.diffuse_color = mat.rgba
        material.blend_method = 'BLEND' if mat.rgba[3] < 1.0 else 'OPAQUE'

        tid = mat.tids[0] if len(mat.tids) else -1

        if tid in images:
            name, image = images[tid]
            texture = bpy.data.textures.new(name=name, type='IMAGE')
            material.use_nodes = True
            bsdf = material.node_tree.nodes["Principled BSDF"]
            texImage = material.node_tree.nodes.new('ShaderNodeTexImage')
            texImage.image = image
            material.node_tree.links.new(bsdf.inputs['Base Color'], texImage.outputs['Color'])

    global imported_armatures, weighting
    imported_armatures = []
    weighting = {}
    imported_armature_objects = []

    import_node_recursive(data)
    make_armatures()
    import_animations(data)

def load(operator,
         context,
         filepath="",
         constrain_size=0.0,
         use_image_search=True,
         use_apply_transform=True,
         global_matrix=None,
         ):

    load_b3d(filepath,
             context,
             IMPORT_CONSTRAIN_BOUNDS=constrain_size,
             IMAGE_SEARCH=use_image_search,
             APPLY_MATRIX=use_apply_transform,
             global_matrix=global_matrix,
             )

    return {'FINISHED'}
