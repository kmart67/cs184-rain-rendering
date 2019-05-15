import bpy
import bmesh
import math

"""
This script is an example of creating and using shapekeys with bmesh
"""
def vert1For(u, t):
    return [ 0, u, 0]

def vert2For(u, t, dTheta, z1):
    theta1 = dTheta * math.sin(t+u*0.4)
    return [ math.sin(theta1)*z1, u, math.cos(theta1)*z1]

def vert3For(u, t, dTheta, z2, thetaLag):
    theta2 = dTheta * math.sin(t+u*0.4-thetaLag)
    return [ math.sin(theta2)*z2, u, math.cos(theta2)*z2]

def makeMesh(name, nSegs, z1, z2, dTheta, thetaLag):
    mesh = bpy.data.meshes.new(name)
    verts = []
    faces = []
    for u in range(0,nSegs+1):
        v4=len(verts)
        verts.append( vert1For(u,0) )
        verts.append( vert2For(u,0,dTheta, z1) )
        verts.append( vert3For(u,0,dTheta, z2, thetaLag) )
        if (u>0):
            v1 = v4-3
            v2 = v1+1
            v3 = v1+2
            v5 = v1+4
            v6 = v1+5
            faces.append( [ v1, v4, v5, v2] )
            faces.append( [ v2, v5, v6, v3] )
    mesh.from_pydata(verts, [], faces)
    mesh.validate(True)
    mesh.show_normal_face = True

    return mesh

def addShapeKey(obj, i, nKeys, z1, z2, dTheta, thetaLag):
    kn = "phase %d"%i
    sk = obj.shape_key_add(kn)
#    sk.value = 0
#    sk.frame = i/nKeys
#    sk.frame = i*i/(nKeys*nKeys) # crazy version

    # YOU NEED TO CREATE A NEW BMESH WHENEVER YOU CREATE A NEW SHAPEKEY
    # THIS IS BECAUSE WHEN YOU DO OBJ.SHAPE_KEY_ADD THE OBJ CHANGES!!
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    sl = bm.verts.layers.shape.get(kn)


    for u in range( math.floor(len(bm.verts) / 3)):
        t = math.pi*2*i/nKeys

        # NOTE HERE: you are indexing into sl from above
        bm.verts[u*3][sl] = vert1For(u, t)
        bm.verts[u*3+1][sl] = vert2For(u, t, dTheta, z1)
        bm.verts[u*3+2][sl] = vert3For(u, t, dTheta, z2, thetaLag)

    bm.to_mesh(obj.data)



dTheta = 0.8
thetaLag = 0.2
z1 = 2
z2 = 3
mesh = makeMesh("fin", 40, z1, z2, dTheta, thetaLag)

obj = bpy.data.objects.new("fin", mesh)
bpy.context.scene.objects.link(obj)


sk0 = obj.shape_key_add("Basis")
print(sk0)
sk0 = obj.data.shape_keys
print(sk0)

# use_relative = False means we use absolute positioning, eg all relative
# to the original basis. otherwise it's per step
sk0.use_relative = False
nKeys = 11;
for i in range(1,nKeys):
    addShapeKey(obj, i, nKeys, z1, z2, dTheta, thetaLag)
    
def dump(mesh):

    bm = bmesh.new()
    bm.from_mesh(mesh)

    bm.verts.ensure_lookup_table()

    for key in bm.verts.layers.shape.keys():
        val = bm.verts.layers.shape.get(key)
        print("%s = %s" % (key,val) )
        sk=mesh.shape_keys.key_blocks[key]
        print("v=%f of [%f .. %f], f=%f, g=%r, b=%r" % ( sk.value, sk.slider_min, sk.slider_max,
                                                       sk.frame, sk.vertex_group, sk.relative_key.name))
        for i in range(len(bm.verts)):
            v = bm.verts[i]
            delta = v[val] - v.co
            if (delta.length > 0):
                print ( "v[%d]+%s" % ( i,delta) )


# Now keyframe the shapekeys
bpy.context.scene.objects.active = bpy.data.objects['fin']
#dump(bpy.context.active_object.data)

me = bpy.context.active_object.data
num_sks = len(me.shape_keys.key_blocks)

#me.shape_keys.eval_time = 50

#me.shape_keys.key_blocks["Basis"].value = 0
me.shape_keys.key_blocks["Basis"].keyframe_insert("value", frame=10)

me.shape_keys.key_blocks["phase 3"].value = 0
me.shape_keys.key_blocks["phase 3"].keyframe_insert("value", frame=30)
    
