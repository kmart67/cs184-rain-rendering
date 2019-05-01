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

    droplet = bpy.data.objects['Droplet']

    fall_and_collide([droplet], bpy.data.objects['Floor'])

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

    obj = bpy.context.active_object
    mat = obj.matrix_world

    # Animate all droplets in the scene.
    for index in range(len(droplets)):
        # Create animation for this droplet.
        action = bpy.data.actions.new("DropletAnimation[%d]" % index)
        mesh = droplets[index].data
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

                vertex_position = mat * v.co
                d = mathutils.geometry.distance_point_to_plane(vertex_position, p1, plane_normal)
                if d < 0:
                    co_kf = co_kf - plane_normal * plane_normal.dot(co_kf - p1)

                insert_keyframe(fcurves, i, co_kf)

    # # Iterate over all droplets to check for collisions.
    # for d in droplets:
    #     # Track the largest necessary displacement to prevent droplet from going through plane.
    #     # largest_offset = -1
    #     for vertex in d.data.vertices:
    #         position = vertex.co
    #         last_position = vertex.last_position
    #
    #         # Move droplet downwards.
    #         position = (position.x, position.y, position.z - GRAVITY)
#
            # If the droplet has crossed the plane, store the needed displacement.
        #     if (normal.dot(position - point) > 0) != (normal.dot(last_position - point) > 0):
        #         tangent_point = position - normal * normal.dot(position - point);
        #         offset = (tangent_point - last_position) * (1.0 - FRICTION) + normal * SURFACE_OFFSET
        #         if offset > largest_offset:
        #             largest_offset = offset
        # if largest_offset > 0:
        #     for vertex in d.data.vertices:
        #         vertex.co = position + largest_offset
        #         vertex.keyframe_insert(data_path="co", index=-1)

class SurfacePlane:
    def __init__(self, friction):
        self.friction

initscene()
