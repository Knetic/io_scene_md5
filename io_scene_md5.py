bl_info = {
    "name": "id tech 4 MD5 format",
    "author": "nemyax",
    "version": (0, 7, 20130705),
    "blender": (2, 6, 6),
    "location": "File > Import-Export",
    "description": "Export md5mesh and md5anim",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Import-Export"}

import bpy
import bmesh
import os.path
import mathutils
import math
from bpy.props import (
    BoolProperty,
    FloatProperty,
    StringProperty,
    IntProperty)
from bpy_extras.io_utils import (
    ExportHelper,
    ImportHelper,
    path_reference_mode)

msgLines = [] # global for error messages
prerequisites = None # global for exportable objects
md5Layer = IntProperty(
        name="Bone Layer",
        description="Bone layer reserved for MD5 export",
        min=1, max=20,
        default=5)
bpy.types.Scene.md5_bone_layer = md5Layer

###
### Export functions
###

def get_ranges(markerFilter):
    markers = bpy.context.scene.timeline_markers
    starts = [m for m in markers if
        m.name.startswith(markerFilter) and
        m.name.endswith("_start", 2)]
    ends = [m for m in markers if
        m.name.startswith(markerFilter) and
        m.name.endswith("_end", 2)]
    if not starts or not ends:
        return None
    else:
        return find_matches(starts, ends)
    
def find_matches(starts, ends):
    pairs = {}
    for s in starts:
        basename = s.name[:s.name.rfind("_start")]
        matches = [e for e in ends if
            e.name[:e.name.rfind("_end")] == basename]
        if matches:
            m = matches[0]
            pairs[basename] = (min(s.frame, m.frame), max(s.frame, m.frame))
    return pairs

def record_parameters(correctionMatrix):
    return "".join([
        " // Parameters used during export:",
        " Reorient: {};".format(bool(correctionMatrix.to_euler()[2])),
        " Scale: {}".format(correctionMatrix.decompose()[2][0])])

def define_components(obj, bm, bones, correctionMatrix):
    scaleFactor = correctionMatrix.to_scale()[0]
    armature = [a for a in bpy.data.armatures if bones[0] in a.bones[:]][0]
    armatureObj = [o for o in bpy.data.objects if o.data == armature][0]
    boneNames = [b.name for b in bones]
    allVertGroups = obj.vertex_groups[:]
    weightGroupIndexes = [vg.index for vg in allVertGroups if vg.name in boneNames]
    uvData = bm.loops.layers.uv.active
    weightData = bm.verts.layers.deform.active
    tris = [[f.index, f.verts[2].index, f.verts[1].index, f.verts[0].index]
        for f in bm.faces] # reverse vert order to flip normal
    verts = []
    weights = []
    wtIndex = 0
    firstWt = 0
    for vert in bm.verts:
        vGroupDict = vert[weightData]
        wtDict = dict([(k, vGroupDict[k]) for k in vGroupDict.keys()
            if k in weightGroupIndexes])
        u = vert.link_loops[0][uvData].uv.x
        v = 1 - vert.link_loops[0][uvData].uv.y # MD5 wants it flipped
        numWts = len(wtDict.keys())
        verts.append([vert.index, u, v, firstWt, numWts])
        wtScaleFactor = 1.0 / sum(wtDict.values())
        firstWt += numWts
        for vGroup in wtDict:
            bone = [b for b in bones
                if b.name == allVertGroups[vGroup].name][0]
            boneIndex = bones.index(bone)
            coords4d =\
                bone.matrix_local.inverted() *\
                armatureObj.matrix_world.inverted() *\
                obj.matrix_world *\
                (vert.co.to_4d() * scaleFactor)
            x, y, z = coords4d[:3]
            weight = wtDict[vGroup] * wtScaleFactor
            wtEntry = [wtIndex, boneIndex, weight, x, y, z]
            weights.append(wtEntry)
            wtIndex += 1
    return (verts, tris, weights)

def make_hierarchy_block(bones, boneIndexLookup):
    block = ["hierarchy {\n"]
    xformIndex = 0
    for b in bones:
        if b.parent:
            parentIndex = boneIndexLookup[b.parent.name]
        else:
            parentIndex = -1
        block.append("  \"{}\" {} 63 {} //\n".format(
            b.name, parentIndex, xformIndex))
        xformIndex += 6
    block.append("}\n")
    block.append("\n")
    return block

def make_baseframe_block(bones, correctionMatrix):
    block = ["baseframe {\n"]
    armature = bones[0].id_data
    armObject = [o for o in bpy.data.objects
        if o.data == armature][0]
    armMatrix = armObject.matrix_world
    for b in bones:
        objSpaceMatrix = b.matrix_local
        if b.parent:
            bMatrix =\
            b.parent.matrix_local.inverted() *\
            armMatrix *\
            objSpaceMatrix
        else:
            bMatrix = correctionMatrix * objSpaceMatrix
        xPos, yPos, zPos = bMatrix.translation
        xOrient, yOrient, zOrient = (-bMatrix.to_quaternion()).normalized()[1:]
        block.append("  ( {:.10f} {:.10f} {:.10f} ) ( {:.10f} {:.10f} {:.10f} )\n".\
        format(xPos, yPos, zPos, xOrient, yOrient, zOrient))
    block.append("}\n")
    block.append("\n")
    return block

def make_joints_block(bones, boneIndexLookup, correctionMatrix):
    block = []
    block.append("joints {\n")
    for b in bones:
        if b.parent:
            parentIndex = boneIndexLookup[b.parent.name]
        else:
            parentIndex = -1
        boneMatrix = correctionMatrix * b.matrix_local
        xPos, yPos, zPos = boneMatrix.translation
        xOrient, yOrient, zOrient =\
        (-boneMatrix.to_quaternion()).normalized()[1:] # MD5 wants it negated
        block.append(\
        "  \"{}\" {} ( {:.10f} {:.10f} {:.10f} ) ( {:.10f} {:.10f} {:.10f} )\n".\
        format(b.name, parentIndex,\
        xPos, yPos, zPos,\
        xOrient, yOrient, zOrient))
    block.append("}\n")
    block.append("\n")
    return block

def make_mesh_block(obj, bones, correctionMatrix):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    triangulate(cut_up(strip_wires(bm)))
    verts, tris, weights = define_components(obj, bm, bones, correctionMatrix)
    bm.free()
    block = []
    block.append("mesh {\n")
    block.append("  shader \"default\"\n")
    block.append("  numverts {}\n".format(len(verts)))
    for v in verts:
        block.append(\
        "  vert {} ( {:.10f} {:.10f} ) {} {}\n".\
        format(v[0], v[1], v[2], v[3], v[4]))
    block.append("  numtris {}\n".format(len(tris)))
    for t in tris:
        block.append("  tri {} {} {} {}\n".format(t[0], t[1], t[2], t[3]))
    block.append("  numweights {}\n".format(len(weights)))
    for w in weights:
        block.append(\
        "  weight {} {} {:.10f} ( {:.10f} {:.10f} {:.10f} )\n".\
        format(w[0], w[1], w[2], w[3], w[4], w[5]))
    block.append("}\n")
    block.append("\n")
    return block

def strip_wires(bm):
    [bm.faces.remove(f) for f in bm.faces if len(f.verts) < 3]
    [bm.edges.remove(e) for e in bm.edges if not e.link_faces[:]]
    [bm.verts.remove(v) for v in bm.verts if v.is_wire]
    for seq in [bm.verts, bm.faces, bm.edges]: seq.index_update()
    return bm

def cut_up(bm):
    uvData = bm.loops.layers.uv.active
    for v in bm.verts:
        for e in v.link_edges:
            linkedFaces = e.link_faces
            if len(linkedFaces) > 1:
                uvSets = []
                for lf in linkedFaces:
                    uvSets.append([l1[uvData].uv for l1 in lf.loops
                        if l1.vert == v][0])
                if uvSets.count(uvSets[0]) != len(uvSets):
                    e.tag = True
                    v.tag = True
        if v.tag:
            seams = [e for e in v.link_edges if e.tag]
            v.tag = False
            bmesh.utils.vert_separate(v, seams)
    for maybeBowTie in bm.verts: # seems there's no point in a proper test
        boundaries = [e for e in maybeBowTie.link_edges
            if len(e.link_faces) == 1]
        bmesh.utils.vert_separate(maybeBowTie, boundaries)
    for seq in [bm.verts, bm.faces, bm.edges]: seq.index_update()
    return bm
      
def triangulate(bm):
    while True:
        nonTris = [f for f in bm.faces if len(f.verts) > 3]
        if nonTris:
            nt = nonTris[0]
            pivotLoop = nt.loops[0]
            allVerts = nt.verts
            vert1 = pivotLoop.vert
            wrongVerts = [vert1,
                pivotLoop.link_loop_next.vert,
                pivotLoop.link_loop_prev.vert]
            bmesh.utils.face_split(nt, vert1, [v for v in allVerts
                if v not in wrongVerts][0])
            for seq in [bm.verts, bm.faces, bm.edges]: seq.index_update()
        else: break
    return bm

def write_md5mesh(filePath, prerequisites, correctionMatrix):
    bones, meshObjects = prerequisites
    boneIndexLookup = {}
    for b in bones:
        boneIndexLookup[b.name] = bones.index(b)
    md5joints = make_joints_block(bones, boneIndexLookup, correctionMatrix)
    md5meshes = []
    for mo in meshObjects:
        md5meshes.append(make_mesh_block(mo, bones, correctionMatrix))
    f = open(filePath, 'w')
    lines = []
    lines.append("MD5Version 10" + record_parameters(correctionMatrix) + "\n")
    lines.append("commandline \"\"\n")
    lines.append("\n")
    lines.append("numJoints " + str(len(bones)) + "\n")
    lines.append("numMeshes " + str(len(meshObjects)) + "\n")
    lines.append("\n")
    lines.extend(md5joints)
    for m in md5meshes: lines.extend(m)
    for line in lines: f.write(line)
    f.close()
    return

def write_md5anim(filePath, prerequisites, correctionMatrix, frameRange):
    goBack = bpy.context.scene.frame_current
    if frameRange == None:
        startFrame = bpy.context.scene.frame_start
        endFrame = bpy.context.scene.frame_end
    else:
        startFrame, endFrame = frameRange
    bones, meshObjects = prerequisites
    armObj = [o for o in bpy.data.objects if o.data == bones[0].id_data][0]
    pBones = armObj.pose.bones
    boneIndexLookup = {}
    for b in bones:
        boneIndexLookup[b.name] = bones.index(b)
    hierarchy = make_hierarchy_block(bones, boneIndexLookup)
    baseframe = make_baseframe_block(bones, correctionMatrix)
    bounds = []
    frames = []
    for frame in range(startFrame, endFrame + 1):
        bpy.context.scene.frame_set(frame)
        verts = []
        for mo in meshObjects:
            bm = bmesh.new()
            bm.from_object(mo, bpy.context.scene)
            verts.extend([correctionMatrix * mo.matrix_world * v.co.to_4d()
                for v in bm.verts])
            bm.free()
        minX = min([co[0] for co in verts])
        minY = min([co[1] for co in verts])
        minZ = min([co[2] for co in verts])
        maxX = max([co[0] for co in verts])
        maxY = max([co[1] for co in verts])
        maxZ = max([co[2] for co in verts])
        bounds.append(\
        "  ( {:.10f} {:.10f} {:.10f} ) ( {:.10f} {:.10f} {:.10f} )\n".\
        format(minX, minY, minZ, maxX, maxY, maxZ))
        frameBlock = ["frame {} {{\n".format(frame - startFrame)]
        scaleFactor = correctionMatrix.to_scale()[0]
        for b in bones:
            pBone = pBones[b.name]
            pBoneMatrix = pBone.matrix
            if pBone.parent:
                diffMatrix = pBone.parent.matrix.inverted() * armObj.matrix_world * (pBoneMatrix * scaleFactor)
            else:
                diffMatrix = correctionMatrix * pBoneMatrix
            xPos, yPos, zPos = diffMatrix.translation
            xOrient, yOrient, zOrient =\
            (-diffMatrix.to_quaternion()).normalized()[1:]
            frameBlock.append(\
            "  {:.10f} {:.10f} {:.10f} {:.10f} {:.10f} {:.10f}\n".\
            format(xPos, yPos, zPos, xOrient, yOrient, zOrient))
        frameBlock.append("}\n")
        frameBlock.append("\n")
        frames.extend(frameBlock)
    f = open(filePath, 'w')
    numJoints = len(bones)
    bounds.insert(0, "bounds {\n")
    bounds.append("}\n")
    bounds.append("\n")
    lines = []
    lines.append("MD5Version 10" + record_parameters(correctionMatrix) + "\n")
    lines.append("commandline \"\"\n")
    lines.append("\n")
    lines.append("numFrames " + str(endFrame - startFrame + 1) + "\n")
    lines.append("numJoints " + str(numJoints) + "\n")
    lines.append("frameRate " + str(bpy.context.scene.render.fps) + "\n")
    lines.append("numAnimatedComponents " + str(numJoints * 6) + "\n")
    lines.append("\n")
    for chunk in [hierarchy, bounds, baseframe, frames]:
        lines.extend(chunk)
    for line in lines:
        f.write(line)
    bpy.context.scene.frame_set(goBack)
    return

def write_batch(filePath, prerequisites, correctionMatrix, markerFilter):
    write_md5mesh(filePath, prerequisites, correctionMatrix)
    ranges = get_ranges(markerFilter)
    if ranges:
        for r in ranges.keys():
            folder = os.path.dirname(filePath)
            animFile = os.path.join(folder, r + ".md5anim")
            write_md5anim(
                animFile, prerequisites, correctionMatrix, ranges[r])
        return {'FINISHED'}
    else:
        baseFilePathEnd = filePath.rfind(".md5mesh")
        if baseFilePathEnd == -1:
            animFilePath = filePath + ".md5anim"
        else:
            animFilePath = filePath[:baseFilePathEnd] + ".md5anim"
        write_md5anim(animFilePath, prerequisites, correctionMatrix, None)
        return {'FINISHED'}

###
### Operators and auxiliary functions
###


# Functions

def concat_strings(strings):
    result = ""
    for s in strings:
        result = result + "\n" + s
    return result

def message(id, *details):
    if id == 'no_deformables':
        return """No armature-deformed meshes are selected.
Select the meshes you want to export, and retry export."""
    elif id == 'multiple_armatures':
        return """The selected meshes use more than one armature.
Select the meshes using the same armature, and try again."""
    elif id == 'no_armature':
        return """No deforming armature is associated with the selection.
Select the model or models you want to export, and try again"""
    elif id == 'layer_empty':
        bl = str(bpy.context.scene.md5_bone_layer)
        return "The deforming armature has no bones in layer " + bl + """.
Add all of the bones you want to export to the armature's layer """ +\
        bl + """,
or change the reserved bone layer in the scene properties,
and retry export."""
    elif id == 'missing_parents':
        bl = str(bpy.context.scene.md5_bone_layer)
        return "One or more bones have parents outside layer " + bl + """.
Revise your armature's layer """ + bl + """ membership,
or change the reserved bone layer in the scene properties, and retry export.
Offending bones:""" + concat_strings(details[0])
    elif id == 'orphans':
        bl = str(bpy.context.scene.md5_bone_layer)
        return """There are multiple root bones (listed below)
in the export-bound collection, but only one root bone
is allowed in MD5. Revise your armature's layer """ + bl + """ membership,
or change the reserved bone layer in the scene properties, and retry export.
Root bones:""" + concat_strings(details[0])
    elif id == 'unweighted_verts':
        if details[0][1] == 1:
            count = " 1 vertex "
        else:
            count = " " + str(details[0][1]) + " vertices "
        return "The '" + details[0][0] + "' object contains" + count +\
        """with no deformation weights assigned.
Valid MD5 data cannot be produced. Paint non-zero weights
on all the vertices in the mesh, and retry export."""
    elif id == 'zero_weight_verts':
        if details[0][1] == 1:
            count = " 1 vertex "
        else:
            count = " " + str(details[0][1]) + " vertices "
        return "The '" + details[0][0] + "' object contains" + count +\
        """with zero weights assigned.
This can cause adverse effects.
Paint non-zero weights on all the vertices in the mesh,
or use the Clean operation in the weight paint tools,
and retry export."""
    elif id == 'no_uvs':
        return "The '" + details[0] + """' object has no UV coordinates.
Valid MD5 data cannot be produced. Unwrap the object
or exclude it from your selection, and retry export."""

def check_weighting(obj, bm, bones):
    boneNames = [b.name for b in bones]
    allVertGroups = obj.vertex_groups[:]
    weightGroups = [vg for vg in allVertGroups if vg.name in boneNames]
    weightGroupIndexes = [vg.index for vg in allVertGroups if vg.name in boneNames]
    weightData = bm.verts.layers.deform.active
    unweightedVerts = 0
    zeroWeightVerts = 0
    for v in bm.verts:
        influences = [wgi for wgi in weightGroupIndexes
            if wgi in v[weightData].keys()]
        if not influences:
            unweightedVerts += 1
        else:
            for wgi in influences:
                if v[weightData][wgi] < 0.000001:
                    zeroWeightVerts += 1
    return (unweightedVerts, zeroWeightVerts)

def is_export_go(what, selection):
    bl = bpy.context.scene.md5_bone_layer - 1
    meshObjects = [o for o in selection
        if o.data in bpy.data.meshes[:] and o.find_armature()]
    armatures = [a.find_armature() for a in meshObjects]
    if not meshObjects:
        return ['no_deformables', None]
    armature = armatures[0]
    if armatures.count(armature) < len(meshObjects):
        return ['multiple_armatures', None]
    bones = [b for b in armature.data.bones if b.layers[bl]]
    if not bones:
        return ['layer_empty', None]
    rootBones = [i for i in bones if not i.parent]
    if len(rootBones) > 1:
        boneList = []
        for rb in rootBones:
            boneList.append("- " + str(rb.name))
        return ['orphans', boneList]
    abandonedBones = [i for i in bones
        if i.parent and i.parent not in bones[:]]
    if abandonedBones:
        boneList = []
        for ab in abandonedBones:
            boneList.append("- " + str(ab.name))
        return ['missing_parents', boneList]
    if what != 'anim':
        for mo in meshObjects:
            bm = bmesh.new()
            bm.from_mesh(mo.data)
            (unweightedVerts, zeroWeightVerts) = check_weighting(mo, bm, bones)
            uvLayer = bm.loops.layers.uv.active
            bm.free()
            if unweightedVerts > 0:
                return ['unweighted_verts', (mo.name, unweightedVerts)]
            if zeroWeightVerts > 0:
                return ['zero_weight_verts', (mo.name, zeroWeightVerts)]
            if not uvLayer:
                return ['no_uvs', mo.name]
    return ['ok', (bones, meshObjects)]

def manage_bone_layers(doWhat):
    bl = bpy.context.scene.md5_bone_layer - 1
    mode = bpy.context.mode
    if mode == 'POSE':
        allBones = [pb.bone for pb in bpy.context.active_object.pose.bones]
        selBones = [pb.bone for pb in bpy.context.selected_pose_bones]
    elif mode == 'EDIT_ARMATURE':
        allBones = bpy.context.active_object.data.edit_bones
        selBones = bpy.context.selected_editable_bones
    else:
        return
    unselBones = [b for b in allBones if b not in selBones]
    if doWhat == 'replace':
        for x in selBones:
            x.layers[bl] = True
        for y in unselBones:
            y.layers[bl] = False
        return
    elif doWhat == 'add':
        for x in selBones:
            x.layers[bl] = True
        return
    elif doWhat == 'remove':
        for x in selBones:
            x.layers[bl] = False
        return
    elif doWhat == 'clear':
        for x in allBones:
            x.layers[bl] = False
        return
    else: return

# Operators

class MD5BonesAdd(bpy.types.Operator):
    '''Add the selected bones to the bone layer reserved for MD5'''
    bl_idname = "scene.md5_bones_add"
    bl_label = 'Add Selected'
    def invoke(self, context, event):
        manage_bone_layers('add')
        return {'FINISHED'}

class MD5BonesRemove(bpy.types.Operator):
    '''Remove the selected bones from the bone layer reserved for MD5'''
    bl_idname = "scene.md5_bones_remove"
    bl_label = 'Remove Selected'
    def invoke(self, context, event):
        manage_bone_layers('remove')
        return {'FINISHED'}

class MD5BonesReplace(bpy.types.Operator):
    '''Include only the selected bones in the bone layer reserved for MD5'''
    bl_idname = "scene.md5_bones_replace"
    bl_label = 'Replace with Selected'
    def invoke(self, context, event):
        manage_bone_layers('replace')
        return {'FINISHED'}

class MD5BonesClear(bpy.types.Operator):
    '''Clear the bone layer reserved for MD5'''
    bl_idname = "scene.md5_bones_clear"
    bl_label = 'Clear All'
    def invoke(self, context, event):
        manage_bone_layers('clear')
        return {'FINISHED'}

class MD5Panel(bpy.types.Panel):
    """MD5 parameters panel in the scene context of the properties editor"""
    bl_label = "MD5 Export Setup"
    bl_idname = "SCENE_PT_md5"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        bl = str(scene.md5_bone_layer)
        layout.prop(scene, "md5_bone_layer")
        column1 = layout.column()
        column1.label("Manage layer " + bl + " membership:")
        column2 = column1.column(align=True)
        column2.operator("scene.md5_bones_add")
        column2.operator("scene.md5_bones_remove")
        column2.operator("scene.md5_bones_replace")
        column2.operator("scene.md5_bones_clear")
        if context.mode in {'POSE','EDIT_ARMATURE'}:
            column1.enabled = True
        else:
            column1.enabled = False

class MaybeMD5Mesh(bpy.types.Operator):
    '''Export selection as MD5 mesh'''
    bl_idname = "export_scene.maybe_md5mesh"
    bl_label = 'Export MD5MESH'
    def invoke(self, context, event):
        global msgLines, prerequisites
        selection = context.selected_objects
        checkResult = is_export_go('mesh', selection)
        if checkResult[0] == 'ok':
            prerequisites = checkResult[-1]
            return bpy.ops.export_scene.md5mesh('INVOKE_DEFAULT')
        else:
            msgLines = message(checkResult[0], checkResult[1])
            print(msgLines)
            self.report({'ERROR'}, msgLines)
            return {'CANCELLED'}

class MaybeMD5Anim(bpy.types.Operator):
    '''Export single MD5 animation (use current frame range)'''
    bl_idname = "export_scene.maybe_md5anim"
    bl_label = 'Export MD5ANIM'
    def invoke(self, context, event):
        global msgLines, prerequisites
        selection = context.selected_objects
        checkResult = is_export_go('anim', selection)
        if checkResult[0] == 'ok':
            prerequisites = checkResult[-1]
            return bpy.ops.export_scene.md5anim('INVOKE_DEFAULT')
        else:
            msgLines = message(checkResult[0], checkResult[1])
            print(msgLines)
            self.report({'ERROR'}, msgLines)
            return {'CANCELLED'}

class MaybeMD5Batch(bpy.types.Operator):
    '''Export a batch of MD5 files'''
    bl_idname = "export_scene.maybe_md5batch"
    bl_label = 'Export MD5 Files'
    def invoke(self, context, event):
        global msgLines, prerequisites
        selection = context.selected_objects
        checkResult = is_export_go('batch', selection)
        if checkResult[0] == 'ok':
            prerequisites = checkResult[-1]
            return bpy.ops.export_scene.md5batch('INVOKE_DEFAULT')
        else:
            msgLines = message(checkResult[0], checkResult[1])
            print(msgLines)
            self.report({'ERROR'}, msgLines)
            return {'CANCELLED'}

class ExportMD5Mesh(bpy.types.Operator, ExportHelper):
    '''Save an MD5 Mesh File'''
    global prerequisites
    bl_idname = "export_scene.md5mesh"
    bl_label = 'Export MD5MESH'
    bl_options = {'PRESET'}
    filename_ext = ".md5mesh"
    filter_glob = StringProperty(
            default="*.md5mesh",
            options={'HIDDEN'},
            )
    path_mode = path_reference_mode
    check_extension = True
    reorient = BoolProperty(
            name="Reorient",
            description="Treat +X as the forward direction",
            default=True,
            )
    scaleFactor = FloatProperty(
            name="Scale",
            description="Scale all data",
            min=0.01, max=1000.0,
            soft_min=0.01,
            soft_max=1000.0,
            default=1.0,
            )
    path_mode = path_reference_mode
    check_extension = True
    def execute(self, context):
        orientationTweak = mathutils.Matrix.Rotation(math.radians(
            -90 * float(self.reorient)),4,'Z')
        scaleTweak = mathutils.Matrix.Scale(self.scaleFactor, 4)
        correctionMatrix = orientationTweak * scaleTweak
        write_md5mesh(self.filepath, prerequisites, correctionMatrix)
        return {'FINISHED'}

class ExportMD5Anim(bpy.types.Operator, ExportHelper):
    '''Save an MD5 Animation File'''
    global prerequisites
    bl_idname = "export_scene.md5anim"
    bl_label = 'Export MD5ANIM'
    bl_options = {'PRESET'}
    filename_ext = ".md5anim"
    filter_glob = StringProperty(
            default="*.md5anim",
            options={'HIDDEN'},
            )
    path_mode = path_reference_mode
    check_extension = True
    reorient = BoolProperty(
            name="Reorient",
            description="Treat +X as the forward direction",
            default=True,
            )
    scaleFactor = FloatProperty(
            name="Scale",
            description="Scale all data",
            min=0.01, max=1000.0,
            soft_min=0.01,
            soft_max=1000.0,
            default=1.0,
            )
    path_mode = path_reference_mode
    check_extension = True
    def execute(self, context):
        orientationTweak = mathutils.Matrix.Rotation(math.radians(
            -90 * float(self.reorient)),4,'Z')
        scaleTweak = mathutils.Matrix.Scale(self.scaleFactor, 4)
        correctionMatrix = orientationTweak * scaleTweak
        write_md5anim(self.filepath, prerequisites, correctionMatrix, None)
        return {'FINISHED'}

class ExportMD5Batch(bpy.types.Operator, ExportHelper):
    '''Save MD5 Files'''
    global prerequisites
    bl_idname = "export_scene.md5batch"
    bl_label = 'Export MD5 Files'
    bl_options = {'PRESET'}
    filename_ext = ".md5mesh"
    filter_glob = StringProperty(
            default="*.md5mesh",
            options={'HIDDEN'},
            )
    path_mode = path_reference_mode
    check_extension = True
    markerFilter = StringProperty(
            name="Marker filter",
            description="Export only frame ranges tagged with "\
            + "markers whose names start with this",
            default="",
            )
    reorient = BoolProperty(
            name="Reorient",
            description="Treat +X as the forward direction",
            default=True,
            )
    scaleFactor = FloatProperty(
            name="Scale",
            description="Scale all data",
            min=0.01, max=1000.0,
            soft_min=0.01,
            soft_max=1000.0,
            default=1.0,
            )
    path_mode = path_reference_mode
    check_extension = True
    def execute(self, context):
        orientationTweak = mathutils.Matrix.Rotation(math.radians(
            -90 * float(self.reorient)),4,'Z')
        scaleTweak = mathutils.Matrix.Scale(self.scaleFactor, 4)
        correctionMatrix = orientationTweak * scaleTweak
        write_batch(
                self.filepath,
                prerequisites,
                correctionMatrix,
                self.markerFilter)
        return {'FINISHED'}

def menu_func_export_mesh(self, context):
    self.layout.operator(
        MaybeMD5Mesh.bl_idname, text="MD5 Mesh (.md5mesh)")
def menu_func_export_anim(self, context):
    self.layout.operator(
        MaybeMD5Anim.bl_idname, text="MD5 Animation (.md5anim)")
def menu_func_export_batch(self, context):
    self.layout.operator(
        MaybeMD5Batch.bl_idname, text="MD5 (batch export)")

def register():
    bpy.utils.register_module(__name__)
    bpy.types.INFO_MT_file_export.append(menu_func_export_mesh)
    bpy.types.INFO_MT_file_export.append(menu_func_export_anim)
    bpy.types.INFO_MT_file_export.append(menu_func_export_batch)

def unregister():
    bpy.utils.unregister_module(__name__)
    bpy.types.INFO_MT_file_export.remove(menu_func_export_mesh)
    bpy.types.INFO_MT_file_export.remove(menu_func_export_anim)
    bpy.types.INFO_MT_file_export.remove(menu_func_export_batch)

if __name__ == "__main__":
    register()
