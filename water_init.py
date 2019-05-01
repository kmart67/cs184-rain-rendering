import bpy

bpy.context.scene.render.engine = 'CYCLES'

#adding sphere
bpy.ops.mesh.primitive_uv_sphere_add(location=(0,0,2))
water = bpy.context.active_object
water.name = 'Water'
bpy.ops.transform.resize(value=(0.5, 0.5, 0.5))
bpy.ops.object.shade_smooth()

#adding water material to the sphere
bpy.context.scene.objects.active = None
bpy.context.scene.objects.active = bpy.data.objects["Water"]

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

#adding plane to the scene
bpy.ops.mesh.primitive_plane_add(location=(0,0,0))
