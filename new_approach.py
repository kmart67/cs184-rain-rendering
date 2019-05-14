import bpy
import bmesh

import math
import mathutils

import numpy as np

addsphere = bpy.ops.mesh.primitive_uv_sphere_add
addplane = bpy.ops.mesh.primitive_plane_add
addcube = bpy.ops.mesh.primitive_cube_add
scene = bpy.context.scene


# global constants
SURFACE_OFFSET = 0.0001
FRICTION_COEFF = 0.0001
GRAVITY = mathutils.Vector((0.0, 0.0, -9.8))
ADVANCING_ANGLE = 90
RECEDING_ANGLE = 30
MASS = 1.0
ALPHA = 1.0
FLOW_COEFFICIENT = 0.1
MU = 0.4
NU = 0.1

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

    # Triangulate meshes.
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.quads_convert_to_tris(quad_method='FIXED', ngon_method='BEAUTY')
    bpy.ops.object.mode_set(mode='OBJECT')

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
#
# """
# Detects collisions between plane and vertex
# """
# def detect_collision(x_old, x_new, p1, plane_normal, mat):
#     # dist_prev = mathutils.geometry.distance_point_to_plane(mat * x_old, p1, plane_normal)
#     dist_curr = mathutils.geometry.distance_point_to_plane(mat * x_new, p1, plane_normal)
#
#     # if (dist_prev > 0) == (dist_curr > 0):
#     #     return (False, x_new)
#
#     # Use only magnitude of distance.
#     if dist_curr < 0:
#         surface_point = x_new - plane_normal * abs(dist_curr)
#     else:
#         return (False, x_new)
#     # surface_point = mat.inverted() * mathutils.geometry.intersect_line_plane(mat * x_old, mat * x_new, p1, plane_normal)
#     assert surface_point != None
#     return (True, surface_point)

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
Calculate L1 norm.
"""
def l1_norm(v):
    return sum([abs(el) for el in v])

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
    # Get same attribute values as were initialized in {@init_scene()}.
    mesh = layer_dict['mesh']
    bm = layer_dict['bmesh']
    collided = layer_dict['collided']



    # Construct plane normal.
    p1 = plane.matrix_world * plane.data.vertices[0].co
    p2 = plane.matrix_world * plane.data.vertices[1].co
    p3 = plane.matrix_world * plane.data.vertices[2].co
    plane_normal = mathutils.geometry.normal([p1, p2, p3])



    # Animate all droplets in the scene.
    end = False
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

        # Calculate volume of mesh.
        volume = bm.calc_volume()

        # Iterate over all vertices in this droplet's mesh and change their positions
        # based on deformation operators.
        vertex_info = {}
        curves_dict = {}
        for i in range(frames):
            if end:
                break

            # Step 4.1: External Forces
            all_verts = bm.verts[:]
            collided_vertices = []
            old_volume = volume
            for v in bm.verts:
                # Create keyframes for this vertex.
                co_kf = v.co

                if i == 0:
                    fcurves = [action.fcurves.new(data_path % v.index, j) for j in range(3)]
                    curves_dict[v.index] = fcurves
                    vertex_info[v] = [mathutils.Vector((0.0, 0.0, 0.0)), mathutils.Vector((0.0, 0.0, 0.0)), v.co, v.co]

                # Identify latest velocity and position, for this vertex.
                _, v_old, _, x_old = vertex_info[v]

                # if coll:
                #     x_new = x_old
                #     v_new = v_old
                #     # Save information to insert keyframe later.
                #     if i in keyframe_collisions:
                #         keyframe_collisions[i][v] = (x_old, x_new, v_old, v_new, detected)
                #     else:
                #         keyframe_collisions[i] = {v: (x_old, x_new, v_old, v_new, detected)}
                #     continue

                # Apply gravitational force.
                v_new = v_old + GRAVITY * dt
                x_new = x_old + v_new * dt

                # Check for collisions.
                detected, x_new = detect_collision(x_new, p1, plane_normal, mat)

                if detected:
                    all_verts.remove(v)
                    v[collided] = True
                    collided_vertices.append(v)

                    # Perform position/velocity update for vertices that have collided with the surface.
                    v_s = (x_new - x_old) / dt

                    # Update velocity (inelastic method).
                    v_old = v_old - (v_old - v_s).dot(plane_normal) * plane_normal

                    # Apply friction force.
                    if l1_norm(v_old) < FRICTION_COEFF:
                        v_new = mathutils.Vector((0.0, 0.0, 0.0))
                    else:
                        v_new = v_old - FRICTION_COEFF * v_old / l1_norm(v_old)

                    # Perform position update.
                    x_new = x_new + (v_new - v_old) * dt

                    # Viscosity update.
                    v_new = (1 - MU * dt) * v_old + NU * dt * v_old

                # Update {@vertex_info}.
                vertex_info[v] = (v_old, v_new, x_old, x_new)

            #     v.co = x_new
            #
            # bm.to_mesh(mesh)

            # Step 4.2: Mean Curvature Flow
            # Calculate mean curvature flow.
            # bmesh.ops.smooth_laplacian_vert(bm, verts=all_verts, lambda_factor=5, lambda_border=0, preserve_volume=True)
            # num_vertices = len(bm.verts)
            #
            # # Create identity matrix.
            # identity = np.identity(num_vertices)
            #
            # # Create lumped mass matrix.
            # lumped_mass_matrix = np.identity(num_vertices) * MASS
            #
            # # Create old position matrix.
            # X_old = np.zeros((num_vertices, 3))
            # index_map = {}
            # inv_index_map = {}
            # bm.verts.ensure_lookup_table()
            # for ind in range(len(bm.verts)):
            #     index_map[bm.verts[ind]] = ind
            #     inv_index_map[ind] = bm.verts[ind]
            #     X_old[ind] = vertex_info[bm.verts[ind]][3]
            #
            # # Create Laplacian-Beltrami matrix.
            # lap_bel_matrix = np.zeros((num_vertices, num_vertices))
            # for ind in range(len(bm.verts)):
            #     v = bm.verts[ind]
            #     curr_vert_index = index_map[v]
            #     sum_res = 0.0
            #     for j in range(len(v.link_edges)):
            #         edge = v.link_edges[j]
            #         neighbor = edge.verts[0] if edge.verts[0] != v else edge.verts[1]
            #         same_faces = [f for f in neighbor.link_faces if f in v.link_faces]
            #         angles = []
            #         for f in same_faces:
            #             for corner in f.loops:
            #                 if corner.vert != neighbor and corner.vert != v:
            #                     angles.append(corner.calc_angle())
            #         alpha_ij = angles[0]
            #         beta_ij = angles[1]
            #         sum_angles = 1.0 / (math.tan(alpha_ij) + 1e-10) + 1.0 / (math.tan(beta_ij) + 1e-10)
            #         res = -0.5 * sum_angles
            #         sum_res += res
            #
            #         # Recall that matrix is symmetric.
            #         neighbor_vert_index = index_map[neighbor]
            #         lap_bel_matrix[curr_vert_index, neighbor_vert_index] = res
            #         lap_bel_matrix[neighbor_vert_index, curr_vert_index] = res
            #     lap_bel_matrix[curr_vert_index, curr_vert_index] = -sum_res
            #
            # assert np.allclose(lap_bel_matrix, lap_bel_matrix.T)
            #
            # # Solve for new vertex positions.
            # multiplier = identity + FLOW_COEFFICIENT * dt * np.dot(np.linalg.inv(lumped_mass_matrix), lap_bel_matrix)
            # X_new = np.dot(np.linalg.inv(multiplier), X_old)
            #
            # for ind in range(X_new.shape[0]):
            #     v = inv_index_map[ind]
            #     v_old, v_new, x_old, x_new = vertex_info[v]
            #     updated_position = X_new[ind]
            #     x_new = mathutils.Vector((updated_position[0], updated_position[1], updated_position[2]))
            #     v_new = v_old + (x_new - x_old) / dt
            #     vertex_info[v] = v_old, v_new, x_old, x_new
            #
            #     v.co = x_new
            #
            # bm.to_mesh(mesh)

            # Step 4.3: Contact Angle Operator
            for v in collided_vertices:
                v_old, v_new, x_old, x_new = vertex_info[v]

                # if detected:
                face_areas = {}
                face_normals = {}
                seen = set()
                seen.add(v)

                num_not_collided = 0
                for f in v.link_faces:
                    num_collided = 1

                    # Check that other vertices are not collided.
                    for v_other in f.verts:
                        # _, _, _, _, other_collided = keyframe_collisions[i][v_other]
                        if not v_other in seen:
                            if v_other in collided_vertices:
                                num_collided += 1
                            if not v_other in collided_vertices:
                                num_not_collided += 1

                            seen.add(v_other)


                    # Make sure the face is not a collapsed one (it's a water-air face).
                    # If the face is collapsed, then all vertices are collided and
                    # num_collided == 2.
                    if num_collided < 3:
                        face_normals[f] = f.normal
                        face_areas[f] = f.calc_area()
                    #
                    # # Exit once we have found the three faces.
                    # if len(face_areas) == 3:
                    #     break

                print(len(face_areas))

                # NOTE: THIS MUST HOLD FOR ALL VERTICES ON THE CONTACT LINE.
                if len(face_areas) == 3:
                    print('hit cond')
                    # Special logic for contact line vertices.

                    # Calculate area-weighted surface normal.
                    n_l = mathutils.Vector((0.0, 0.0, 0.0))
                    total_area = sum(face_areas.values())
                    for f in face_areas:
                        n_l += face_normals[f] * (face_areas[f] / total_area)

                    # Calculate projection of surface normal onto the plane.
                    print('normal at surface', n_l)
                    n_p = n_l - n_l.dot(plane_normal) * plane_normal

                    # Calculate the bounding force, applied to the contact vertex.
                    f_bound = 0
                    theta = math.degrees(angle(n_l, plane_normal))
                    print(theta)
                    if theta >= RECEDING_ANGLE and theta <= ADVANCING_ANGLE:
                        f_bound = 0
                        end = True
                    elif theta < RECEDING_ANGLE:
                        f_bound = ALPHA * (theta - RECEDING_ANGLE) * n_p / l1_norm(n_p)
                    elif theta > ADVANCING_ANGLE:
                        f_bound = ALPHA * (theta - ADVANCING_ANGLE) * n_p / l1_norm(n_p)

                    if end:
                        break

                    # Change position to match force bound if needed.
                    accel = v_new / dt
                    if accel * MASS > f_bound:
                        bound_vel = f_bound / MASS * dt
                        x_new = x_old + bound_vel * dt
                        v_new = v_old + (x_new - x_old) / dt

                vertex_info[v] = (v_old, v_new, x_old, x_new)

                # v.co = x_new
                # bm.to_mesh(mesh)
                # keyframe_collisions[i][v] = (x_old, x_new, v_old, v_new, detected)

            # Step 4.4: Volume Correction
            for v in bm.verts:
                v_old, v_new, x_old, x_new = vertex_info[v]
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
                v_old, v_new, x_old, x_new = vertex_info[v]
                x_new = x_new - d * v.normal
                vertex_info[v] = (v_old, v_new, x_old, x_new)
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
