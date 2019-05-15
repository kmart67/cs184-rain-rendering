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
GRAVITY = mathutils.Vector((0.0, 0.0, -9.8))
FRICTION_COEFF = 0.9

#def clearscene():


def initscene():
    #adding the open box
    #right wall
    addplane(location=(1,0,0.75), rotation=(math.radians(90), 0, math.radians(90)))
    bpy.context.active_object.name = 'RightWall'
    bpy.context.object.scale[1] = 0.75
    mat = bpy.data.materials.new('RightWallMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (0, 1, 0)

    #left wall
    addplane(location=(-1,0,0.75), rotation=(math.radians(90), 0, math.radians(90)))
    bpy.context.active_object.name = 'LeftWall'
    bpy.context.object.scale[1] = 0.75
    mat = bpy.data.materials.new('LeftWallMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (1, 0, 0)

    #ceiling wall
    addplane(location=(0,0,1.5))
    bpy.context.active_object.name = 'Ceiling'
    mat = bpy.data.materials.new('CeilingMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (1, 1, 1)

    #floor
    addplane(location=(0,0,0))
    bpy.context.active_object.name = 'Floor'
    mat = bpy.data.materials.new('FloorMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (1, 1, 1)

    #backwall
    addplane(location=(0,-1,0.75), rotation=(math.radians(90), 0, 0))
    bpy.context.active_object.name = 'BackWall'
    bpy.context.object.scale[1] = 0.75
    mat = bpy.data.materials.new('BackWallMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (1, 1, 1)

    #joining together the walls
#   for ob in bpy.context.scene.objects:
#        if ob.type == 'MESH':
#            ob.select = True
#            bpy.context.scene.objects.active = ob
#        else:
#            ob.select = False

#    bpy.ops.object.join()
#    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
#    bpy.data.meshes["Plane"].name = "wall"

    #adding lightsource
    addplane(location=(0,0,1.49))
    bpy.context.active_object.name = 'LightSource'
    bpy.ops.transform.resize(value=(0.45, 0.35, 1))
    bpy.ops.object.lamp_add(type='AREA', location=(0,0,1.5))

    #adding camera
    bpy.ops.object.camera_add(location=(0,3,0), rotation=(math.radians(90), 0, math.radians(180)))
    bpy.context.active_object.name = 'Camera'
    bpy.ops.transform.resize(value=(0.5, 0.5, 0.5))
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

    #adding the glass plane
    addplane(location=(0,0,0.75), rotation=(math.radians(90), 0, 0))
    bpy.context.active_object.name = 'Glass'
    bpy.ops.transform.translate()
    bpy.context.object.scale[0] = 0.5
    bpy.context.object.scale[1] = 0.5
    bpy.context.object.scale[2] = 0.5
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

    # add droplet
    addsphere(location=(0,0,0.1), size=0.05)
    bpy.context.active_object.name = 'Droplet'
    bpy.ops.object.shade_smooth()
    bpy.context.scene.objects.active = bpy.data.objects["Droplet"]

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

    droplet = bpy.data.objects['Droplet']
    layer_dict = add_new_attributes(droplet)

    # Add Basis shapekey
    nKeys = 11
    sk0 = droplet.shape_key_add("Basis")
    sk0 = droplet.data.shape_keys
    sk0.use_relative = False
    # bm = layer_dict['bmesh']

    mesh = droplet.data
    if mesh.is_editmode:
        bm = bmesh.from_edit_mesh(mesh)
    else:
        bm = bmesh.new()
        bm.from_mesh(mesh)

    collided = bm.verts.layers.int.new('collided')
    last_position = bm.verts.layers.shape.new('last_position')
    velocity = bm.verts.layers.shape.new('velocity')

    for v in bm.verts:
        v[collided] = False
        v[last_position] = v.co
        v[velocity] = mathutils.Vector((0.0, 0.0, 0.0))


    for i in range(1, nKeys):
        # Add a shapekey
        kn = "phase %d"%i
        sk = droplet.shape_key_add(kn)
        bm.verts.ensure_lookup_table()
        sl = bm.verts.layers.shape[kn]
        bm.verts[0][sl] = [2, 2, 2]
        fall_and_collide([droplet], bpy.context.scene.objects['Floor'], layer_dict, sl)

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
    last_position = bm.verts.layers.shape.new('last_position')
    velocity = bm.verts.layers.shape.new('velocity')

    for v in bm.verts:
        v[collided] = False
        v[last_position] = v.co
        v[velocity] = mathutils.Vector((0.0, 0.0, 0.0))

    #bm.to_mesh(mesh)

    return {'bmesh': bm, 'mesh': mesh,
            'collided': collided, 'last_position': last_position,
            'velocity': velocity}

"""
Inserts keyframe for mesh modifications.
"""
def insert_keyframe(fcurves, frame, values):
    for fcu, val in zip(fcurves, values):
        fcu.keyframe_points.insert(frame, val, {'FAST'})

"""
Have the droplet fall for one time frame, and check for collisions between any
droplet and the rigid body object inputted.

@droplets   list of UV Sphere instances representing water droplets
@plane      plane against which to check for collision
"""
def fall_and_collide(droplets, plane, layer_dict, sl):
    # Get custom attributes.
    mesh = layer_dict['mesh']
    bm = layer_dict['bmesh']
    collided = layer_dict['collided']
    last_position = layer_dict['last_position']
    velocity = layer_dict['velocity']

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
        location_datapath = "vertices[%d].co"
        velocity_datapath = "vertices[%d].velocity"
        dt = 0.05

        # Iterate over all vertices in this droplet's mesh and change their positions
        # based on external forces.
        

        # Step 4.1
        for v in bm.verts:
            # Create keyframes for this vertex.
            fcurves_location = [action.fcurves.new(location_datapath % v.index, i) for i in range(3)]
            fcurves_velocity = [action.fcurves.new(velocity_datapath % v.index, i) for i in range(3)]
            x_old = v.co
            v_old = v[velocity]

            for i in range(2):
                # Apply gravity force and use Forward Euler for position update.
                v_new = v_old + GRAVITY * dt
                x_new = x_old + v_new * dt

                # Check distance to plane in world coordinates.
                vertex_position = mat * x_new
                d = mathutils.geometry.distance_point_to_plane(vertex_position, p1, plane_normal)

                # Check for collision.
                if d < 0:
                    print('collision')
                    # If collision, project vertex to closest point on plane surface
                    # and update velocity. Ignore viscosity for now.
                    v[collided] = True

                    # Find surface projection and velocity at surface.
                    x_surface = x_new - plane_normal * d
                    x_new = x_surface
                    v_s = (x_surface - x_old) / dt
                    surface_normal = mathutils.geometry.normal([p1, p2, x_new])

                    # Update velocity.
                    v_new = v_old - (v_old - v_s).dot(surface_normal) * surface_normal

                    # Apply friction force.
                    # if v_old.magnitude < FRICTION_COEFF:
                    #     v_new = mathutils.Vector((0.0, 0.0, 0.0))
                    # else:
                    #     v_new = v_old - FRICTION_COEFF * v_old / v_old.magnitude

                    # Perform position update.
                    # x_new = x_old + (v_new - v_old) * dt

                bm.to_mesh(mesh)
                # insert_keyframe(fcurves_location, i, x_new)
                # insert_keyframe(fcurves_velocity, i, v_new)
                v[velocity] = v_new
                v[sl] = x_new
                # Update old values.
                x_old = x_new
                v_old = v_new

        # Step 4.3
        contact_verts = []
        for v in bm.verts:
            if v[collided]:
                no_col = True
                for e in v.link_edges:
                    v_other = e.other_vert(v)
                    if v_other[collided]:
                        no_col = False
                        break
                if no_col:
                    contact_verts.append(v)
                    print('faces', v.link_faces)

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
