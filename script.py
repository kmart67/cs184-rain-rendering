import bpy
import math

addsphere = bpy.ops.mesh.primitive_uv_sphere_add
addplane = bpy.ops.mesh.primitive_plane_add
addcube = bpy.ops.mesh.primitive_cube_add
scene = bpy.context.scene

def initscene():
    #adding the open box
    #right wall
    addplane(location=(1,0,0.75), rotation=(math.radians(90), 0, math.radians(90)))
    bpy.context.object.scale[1] = 0.75
    mat = bpy.data.materials.new('RightWallMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (0, 1, 0)

    #left wall
    addplane(location=(-1,0,0.75), rotation=(math.radians(90), 0, math.radians(90)))
    bpy.context.object.scale[1] = 0.75
    mat = bpy.data.materials.new('LeftWallMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (1, 0, 0)

    #ceiling wall
    addplane(location=(0,0,1.5))
    mat = bpy.data.materials.new('CeilingMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (1, 1, 1)

    #floor
    addplane(location=(0,0,0))
    mat = bpy.data.materials.new('FloorMaterial')
    bpy.context.active_object.data.materials.append(mat)
    bpy.context.object.active_material.diffuse_color = (1, 1, 1)
    bpy.ops.rigidbody.objects_add(type='PASSIVE')

    #backwall
    addplane(location=(0,-1,0.75), rotation=(math.radians(90), 0, 0))
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
    bpy.ops.transform.resize(value=(0.45, 0.35, 1))
    bpy.ops.object.lamp_add(type='AREA', location=(0,0,1.5))

    #adding camera
    bpy.ops.object.camera_add(location=(0,3,0), rotation=(math.radians(90), 0, math.radians(180)))
    bpy.ops.transform.resize(value=(0.5, 0.5, 0.5))
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

    #adding the glass plane
    addplane(location=(0,0,0.75), rotation=(math.radians(90), 0, 0))
    bpy.ops.transform.translate()
    bpy.context.object.scale[0] = 0.5
    bpy.context.object.scale[1] = 0.5
    bpy.context.object.scale[2] = 0.5
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

    # add droplet
    addsphere(location=(0,0,1.49), size=0.05)

    droplet = bpy.data.objects["Sphere"]
    bpy.ops.rigidbody.objects_add(type='ACTIVE')


class SurfacePlane:
    def __init__(self, friction):
        self.friction

initscene()
