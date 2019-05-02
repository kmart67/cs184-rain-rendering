import bpy
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
    addsphere(location=(0,0,1.49), size=0.05)
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

    fall_and_collide([droplet], bpy.context.scene.objects['Floor'])

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
def fall_and_collide(droplets, plane):
    # Get random point on plane.
#    point = plane.data.vertices[0]
#    normal = plane.normal

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
        mesh = obj.data
        mesh.animation_data_create()
        mesh.animation_data.action = action
        mesh.animation_data_create()
        mesh.animation_data.action = action
        data_path = "vertices[%d].co"
        dt = 0.01

        # Iterate over all vertices in this droplet's mesh and change their positions
        # based on external forces.
        for v in mesh.vertices:
            # Create keyframes for this vertex.
            fcurves = [action.fcurves.new(data_path % v.index, i) for i in range(3)]
            co_kf = v.co
            velocity = mathutils.Vector((0.0, 0.0, 0.0))

            for i in range(70):
                co_kf = co_kf + velocity * dt
                velocity.z -= GRAVITY * dt

                vertex_position = mat * co_kf
                d = mathutils.geometry.distance_point_to_plane(vertex_position, p1, plane_normal)
                if d < 0:
                    co_kf = co_kf - plane_normal * d
                insert_keyframe(fcurves, i, co_kf)

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
