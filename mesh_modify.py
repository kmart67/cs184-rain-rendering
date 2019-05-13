# This example assumes we have a mesh object in edit-mode

import bpy
import bmesh
import mathutils

# Get the active mesh
obj = bpy.context.edit_object
me = obj.data


# Get a BMesh representation
bm = bmesh.from_edit_mesh(me)
bm.faces.active = None

# Create a new layer (eg, instance attr for each vert) for velocity
velocity = bm.verts.layers.shape.new('v')

### GLOBAL VARS ###
dt = 0.1
epsilon = 0.5
mu = 0.4
viscosity = 0.05

# Modify the BMesh, can do anything here...
for v in bm.verts:
    
    # example: shift each vector position (co) by .1
    v.co += mathutils.Vector((.1, .1, .1))
    
    # example: set each velocity equal to the current vertex position
    v[velocity] = v.co


    # forward euler with gravity
    v[velocity] = v[velocity] - 9.8 * dt
    v.co = v.co + v[velocity]*dt
 
# Prints all vertex velocities.  
for v in bm.verts:
    print(v[velocity])


# Show the updates in the viewport
# and recalculate n-gon tessellation.
bmesh.update_edit_mesh(me, True)
