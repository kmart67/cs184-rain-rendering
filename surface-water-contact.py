import bpy
import bmesh

import math
import mathutils

addsphere = bpy.ops.mesh.primitive_uv_sphere_add
addplane = bpy.ops.mesh.primitive_plane_add
addcube = bpy.ops.mesh.primitive_cube_add
scene = bpy.context.scene

# global constants
SURFACE_OFFSET = 0.0001
FRICTION = 0.0001
GRAVITY = 9.8
ADVANCING_ANGLE = 90
RECEDING_ANGLE = 30
MASS = 1.0
ALPHA = 1.0

def initscene():
    #floor
    addplane(location=(0,0,0))
    bpy.context.active_object.name = 'Floor'
    mat = bpy.data.materials.new('FloorMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (1, 1, 1)

    #adding camera
    bpy.ops.object.camera_add(location=(0,3,0), rotation=(math.radians(90), 0, math.radians(180)))
    bpy.context.active_object.name = 'Camera'
    bpy.ops.transform.resize(value=(0.5, 0.5, 0.5))
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

    #adding the glass plane
    # addplane(location=(0,0,0.75), rotation=(math.radians(90), 0, 0))
    # bpy.context.active_object.name = 'Glass'
    # bpy.ops.transform.translate()
    # bpy.context.object.scale[0] = 0.5
    # bpy.context.object.scale[1] = 0.5
    # bpy.context.object.scale[2] = 0.5
    # bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

    # add droplet
    addsphere(location=(0,0,0.3), size=0.05)
    name = "Droplet1"
    bpy.context.active_object.name = name
    bpy.ops.object.shade_smooth()
    bpy.context.scene.objects.active = bpy.data.objects[name]

    ob = bpy.context.active_object

    # Create material
    mat = bpy.data.materials.new(name="MaterialWater")

    # Assign it to object
    if len(ob.data.materials):
        # assign to 1st material slot
        ob.data.materials[0] = mat
    else:
        # no solots
        ob.data.materials.append(mat)

    # Activated material -> cmat
    cmat = ob.active_material
    cmat.use_nodes = True
    TreeNodes = cmat.node_tree
    links = TreeNodes.links

    # Remove nodes (clean it)
    for node in TreeNodes.nodes:
        TreeNodes.nodes.remove(node)

    # Add the guy to the node view
    # Output node
    node_out = TreeNodes.nodes.new(type='ShaderNodeOutputMaterial')
    node_out.location = 200, 0

    # Glass BSDF
    node_glass = TreeNodes.nodes.new(type='ShaderNodeBsdfGlass')
    node_glass.location = 0, 180
    node_glass.distribution = 'GGX'
    node_glass.inputs['Color'].default_value = (1.0, 1.0, 1.0, 1)
    node_glass.inputs['Roughness'].default_value = 0.0
    node_glass.inputs['IOR'].default_value = 1.330

    # Connect the guys
    links.new(node_glass.outputs[0], node_out.inputs[0])

    droplet1 = bpy.data.objects['Droplet1']
    layer_dict = add_new_attributes(droplet1)

    fall_and_collide([droplet1], bpy.context.scene.objects['Floor'], layer_dict)

"""
Add new droplet attributes to the provided instance.
"""
def add_new_attributes(droplet):
    mesh = droplet.data
    if mesh.is_editmode:
        bm = bmesh.from_edit_mesh(mesh)
    else:
        bm = bmesh.new()
        bm.from_mesh(mesh)

    collided = bm.verts.layers.int.new('collided')

    for v in bm.verts:
        v[collided] = False

    bm.to_mesh(mesh)

    return {'bmesh': bm, 'mesh': mesh, 'collided': collided}

"""
Inserts keyframe for mesh modifications.
"""
def insert_keyframe(fcurves, frame, values):
    for fcu, val in zip(fcurves, values):
        fcu.keyframe_points.insert(frame, val, {'FAST'})

"""
Detects collisions between plane and vertex
"""
def detect_collision(co_kf, p1, plane_normal, mat):
    vertex_position = mat * co_kf
    d = mathutils.geometry.distance_point_to_plane(vertex_position, p1, plane_normal)
    if d < 0:
        co_kf = co_kf - plane_normal * d
        return True, co_kf
    return False, co_kf

"""
Dot product between two vectors.
"""
def dot(v1, v2):
    return sum((a*b) for a, b in zip(v1, v2))

"""
Length of a vector
"""
def length(v):
  return math.sqrt(dot(v, v))

"""
Angle between two vectors
"""
def angle(v1, v2):
  return math.acos(dot(v1, v2) / (length(v1) * length(v2)))

"""
Have the droplet fall for one time frame, and check for collisions between any
droplet and the rigid body object inputted.

@droplets   list of UV Sphere instances representing water droplets
@plane      plane against which to check for collision
"""
def fall_and_collide(droplets, plane, layer_dict):
    # Get random point on plane.
#    point = plane.data.vertices[0]
#    normal = plane.normal
    bm = layer_dict['bmesh']
    mesh = layer_dict['mesh']
    collided = layer_dict['collided']

    # Construct plane normal.
    p1 = plane.matrix_world * plane.data.vertices[0].co
    p2 = plane.matrix_world * plane.data.vertices[1].co
    p3 = plane.matrix_world * plane.data.vertices[2].co
    plane_normal = mathutils.geometry.normal([p1, p2, p3])

    # Animate all droplets in the scene.
    for index in range(len(droplets)):
        obj = droplets[index]
        mat = obj.matrix_world

        # Create animation for this droplet.
        action = bpy.data.actions.new("DropletAnimation[%d]" % index)
        mesh.animation_data_create()
        mesh.animation_data.action = action
        data_path = "vertices[%d].co"
        dt = 0.01
        frames = 100
        volume = bm.calc_volume()

        # Iterate over all vertices in this droplet's mesh and change their positions
        # based on external forces.
        vertex_info = {}
        curves_dict = {}
        for i in range(frames):
            # Step 4.1
            collided_vertices = []
            old_volume = volume
            for v in bm.verts:
                # Create keyframes for this vertex.
                co_kf = v.co

                if i == 0:
                    fcurves = [action.fcurves.new(data_path % v.index, j) for j in range(3)]
                    curves_dict[v.index] = fcurves
                    vertex_info[v] = [mathutils.Vector((0.0, 0.0, 0.0)), mathutils.Vector((0.0, 0.0, 0.0)), v.co, v.co]

                _, vel_old, _, x_old = vertex_info[v]
                x_new = x_old + vel_old * dt
                vel = vel_old
                vel.z -= GRAVITY * dt

                detected, x_new = detect_collision(x_new, p1, plane_normal, mat)

                if detected:
                    v[collided] = True
                    collided_vertices.append(v)

                vertex_info[v] = [vel_old, vel, x_old, x_new]

            for v in collided_vertices:

                vel_old, vel, x_old, x_new = vertex_info[v]

                face_areas = {}
                face_normals = {}
                for f in v.link_faces:
                    num_collided = 0

                    # Check that other vertices are not collided.
                    for v_other in f.verts:
                        if v_other[collided]:
                            num_collided += 1

                    # Make sure the face is not a collapsed one (it's a water-air face).
                    # If the face is collapsed, then all vertices are collided and
                    # num_collided = 3.
                    if num_collided != 3:
                        face_normals[f] = f.normal
                        face_areas[f] = f.calc_area()
                    #
                    # # Exit once we have found the three faces.
                    # if len(face_areas) == 3:
                    #     break

                # NOTE: THIS MUST HOLD FOR ALL VERTICES ON THE CONTACT LINE.
                if len(face_areas) == 3:
                    # Special logic for contact line vertices.

                    # Calculate area-weighted surface normal.
                    n_l = mathutils.Vector((0.0, 0.0, 0.0))
                    total_area = sum(face_areas.values())
                    for f in face_areas:
                        n_l += face_normals[f] * (face_areas[f] / total_area)

                    # Calculate projection of surface normal onto the plane.
                    n_p = n_l - n_l.dot(plane_normal) * plane_normal

                    # Calculate the bounding force, applied to the contact vertex.
                    f_bound = 0
                    theta = math.degrees(angle(n_l, plane_normal))
                    if theta > RECEDING_ANGLE and theta < ADVANCING_ANGLE:
                        f_bound = 0
                    elif theta < RECEDING_ANGLE:
                        f_bound = ALPHA * (theta - RECEDING_ANGLE) * n_p / (n_p.magnitude ** 2)
                    elif theta > ADVANCING_ANGLE:
                        f_bound = ALPHA * (theta - ADVANCING_ANGLE) * n_p / (n_p.magnitude ** 2)

                    # Change position to match force bound if needed.
                    accel = vel / dt
                    if accel * MASS > f_bound:
                        bound_vel = f_bound / MASS * dt
                        x_new = x_old + (bound_vel - vel_old) * dt
                        vertex_info[v] = [vel_old, bound_vel, x_old, x_new]

            ## VOLUME CORRECTION CODE
            for v in bm.verts:
                vel_old, vel, x_old, x_new = vertex_info[v]
                v.co = x_new
                bm.to_mesh(mesh)

            volume = bm.calc_volume()            
            area = sum(f.calc_area() for f in bm.faces)
            if area == 0:
                d = 0
            else:
                d = (volume - old_volume) / area
            
            # FINAL VOLUME CORRECTION CHANGES + KEYFRAMING
            for v in bm.verts:
                vel_old, vel, x_old, x_new = vertex_info[v]
                x_new = x_new - d * v.normal
                vertex_info[v] = [vel_old, vel, x_old, x_new]
                insert_keyframe(curves_dict[v.index], i, x_new)


class SurfacePlane:
    def __init__(self, friction):
        self.friction

def delete_scene_objects(scene=None):
    """Delete a scene and all its objects."""
    #
    # Sort out the scene object.
    if scene is None:
        # Not specified: it's the current scene.
        scene = bpy.context.screen.scene
    else:
        if isinstance(scene, str):
            # Specified by name: get the scene object.
            scene = bpy.data.scenes[scene]
        # Otherwise, assume it's a scene object already.
    #
    # Remove objects.
    for object_ in scene.objects:
        bpy.data.objects.remove(object_, True)

# Tests.
#
# Delete the current scene.
delete_scene_objects()

initscene()
