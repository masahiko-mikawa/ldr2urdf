#! /usr/bin/env python

# usage: python3 ldr2urdf.py hoge.ldr

import os, sys, getopt
#import pathlib
#import math
import copy
import numpy
import transformations as TF
import UrdfTemplates as TP

# 1[LDU] = 0.4 [mm]
LDU2MM = 0.4
LDU2M  = 0.4 * 0.001

g_dae_path = "package://ev3_manipulator/meshes/dae/"
g_root = 'map'  # /base_link, map, ...

def inverseHomogeneous(mat):
    # mat is array
    # chech size of mat (whether homogeneous matrix or not)
    numColumn = mat.shape[0]
    numRow = mat.shape[0]
    if numColumn != 4 or numRow != 4:
        print("Size of matrix is wrong")
        return

    R = mat[0:3,0:3]
    T = mat[0:3,3:4]
    Rinv = numpy.linalg.inv(R)
    #Rinv = numpy.matrix.transpose(R)
    RinvT = numpy.dot(Rinv, T)
    Hinv = TF.identity_matrix()     # create 4x4 matrix
    Hinv[0:3,0:3] = Rinv
    Hinv[0:3,3:4] = -RinvT
    return Hinv


class Mimic():
    def __init__(self):
        self.is_mimic = False
        self.joint = ''         # ex: 'ref_205_joint'
        self.multiplier = 1.0
        self.offset = 0.0

class Xyz():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
class Axis():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        
class Rpy():
    def __init__(self):
        self.r = 0.0
        self.p = 0.0
        self.y = 0.0

class Origin():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.r = 0.0
        self.p = 0.0
        self.y = 0.0
        
class DaePart():
    def __init__(self):
        self.name = ''   # ex: 99550c01 is EV3 Large Motor Case
        self.ref_id = -1        # ref_id
        self.ref_link = ''      # 'ref_xxx_link' (xxx is ref_id)
        self.color = 0
        self.rosHdae = numpy.identity(4) # H matrix in ROS coordinate system
        self.linkHdae = numpy.identity(4) # H matrix in link
        self.is_parent_joint = False
        self.is_child_joint = False
        self.is_root = False
        self.joint = Joint()    # joint to parent part
        self.parent_ref_link = ''   # ex: ref_xxx_link (root part's link name in each link)
        self.root_link = ''

    def print(self):
        if self.is_parent_joint == True:
            print("   parent joint")
        if self.is_child_joint == True:
            print("   child joint")
        # print("rHd = ", self.rHd)
        # print("rHd = ", self.lHd)
        # index = 0
        # for i in self.parts:
        #     print("part[%d]:" % index)
        #     i.print()
        #     index += 1
    

class Joint():
    def __init__(self):
        # 0 !LEOCAD GROUP BEGING Joint "id" "joint_type" "parent/child_id" continuous a_x a_y a_z
        self.joint_type = 'fixed'
        self.id = -1            # id defined in ldr by user (not ref_xxx_joint)
        self.parent_id = -1     # ex: parent "id" continuous 1 0 0
        self.child_id = -1      # ex: child "id" continuous 1 0 0
        self.mimic = Mimic()
        self.origin = Origin()
        self.axis = Axis()

class Link():
    def __init__(self):
        # revolute/continuous/prismatic/fixed/floating/planar
        self.parent_id = -1                  # id is defined in ldr
        self.self_id   = -1                  # id is defined in ldr
        self.child_id  = -1                  # id is defined in ldr
        self.rosHlink = numpy.identity(4)    # H matrix in ROS coorinate system
        self.parentHlink = numpy.identity(4) # H matrix from parent coordinate system
        self.parts = []                      # parts other than parents and child
        self.child_joint_part = DaePart()    # child part is always one
        self.root_part_ref_link = ''         # ref_xxx_link
        self.ref_joint = ''                  # ref_xxx_joint
        self.root_part = DaePart()           # root DaePart()
        self.parent_joints_ref_joint = []    # ref_xxx_joint (a link may have multiple joints.)
        self.child_joint = DaePart()
        self.parent_joints = []               # DaeParts() (a link may have multiple joints.)

    def print(self):
        print("  self_id    = %d" % self.self_id)
        print("  parent_id  = %d" % self.parent_id)
        print("  child_id   = %d" % self.child_id)
        #print("rHl = ", self.rHl)
        #print("axis       = (%d, %d, %d)" % (self.axis_x, self.axis_y, self.axis_y))
        #print("rHd = ", self.rHl)
        index = 0
        print("  num of parts = %d" % len(self.parts))
        for p in self.parts:
            #print("parts[%d]:" % index)
            p.print()
            index += 1
            
class Urdf():
    def __init__(self):
        self.name = 'unknow'
        self.is_link  = False
        self.is_parent_joint = False
        self.is_parent_joint_part = False
        self.is_child_joint = False
        self.is_mimic_child_joint = False
        self.links = []
        # scale matrix from [LDU] to [m]
        self.ldu2m = numpy.array([
            [LDU2M, 0.0,   0.0,   0.0],
            [0.0,   LDU2M, 0.0,   0.0],
            [0.0,   0.0,   LDU2M, 0.0],
            [0.0,   0.0,   0.0,   1.0]])
        # H matrix from ROS to ldraw
        self.rosHldraw = numpy.array([
            [1.0,  0.0, 0.0, 0.0],
            [0.0,  0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0,  0.0, 0.0, 1.0]])
        self.tmp_joint = Joint()
        self.tmp_mimic = Mimic()
        
    def print(self):
        #print("name = %s" % self.name)
        index = 0
        print("num of links = %d" % len(self.links))
        for l in self.links:
            print("links[%d]:" % index)
            l.print()
            index += 1
            
    def load(self, ldr_file, urdf_file):
        self.is_link = False
        print("Loading %s ..." % ldr_file, file=sys.stderr)
        if os.path.isfile(ldr_file) != True:
            print("%s not found" % ldr_file, file=sys.stderr)
            return
        
        fp_urdf = open(urdf_file, "w")
        ### xml Header
        print("<?xml version=\"1.0\" ?>", file = fp_urdf)
        print("<!-- This file was generated from %s. -->" % ldr_file, file = fp_urdf)
        print("<robot name=\"%s\">" % urdf_file, file = fp_urdf)
        print("", file = fp_urdf)
        print("    <link name=\"%s\"/>" % g_root, file = fp_urdf)
        
        
        fp_ldr = open(ldr_file, "r")
        lines = fp_ldr.read().splitlines()
        fp_ldr.close()
        
        print("lines.len = %d" % len(lines))
        
        # scale: ldu2m
        # partHdae = (rosHldraw)^(-1)
        # ldrawHpart <- loaded from ldr file
        # ldrawHdae = ldrawHpart * partHdae = ldrawHpart * (rosHldraw)^(-1)
        # ldrawHdae' = ldu2m * ldrawHdae
        # rosHdae = rosHldraw * ldrawHdae'
        
        # rosHdae = rosHlink * linkHdae
        # <-> linkHdae = (rosHlink)^(-1) * rosHdae
        # rosHdae = rosHparent * parentHdae
        # <-> linkHdae = (rosHparent)^(-1) * rosHdae
        
        # scale matrix from [LDU] to [m]
        ldu2m = numpy.array([
            [LDU2M, 0.0,   0.0,   0.0],
            [0.0,   LDU2M, 0.0,   0.0],
            [0.0,   0.0,   LDU2M, 0.0],
            [0.0,   0.0,   0.0,   1.0]])
        # H matrix from ROS to ldraw
        rosHldraw = numpy.array([
            [1.0,  0.0, 0.0, 0.0],
            [0.0,  0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0,  0.0, 0.0, 1.0]])
        # H matrix from part (dat in ldraw) to dae (in ROS)
        partHdae = TF.inverse_matrix(rosHldraw)
        partHdae = inverseHomogeneous(rosHldraw)
        
        ref_id = 0
        link_no = -1
        for line in lines:
            segments = line.split()
            num_seg = len(segments)
            #print("num_seg = %d" % num_seg)
            
            if num_seg==8 and segments[3]=='BEGIN' and segments[4]=='Link':
                self_id   = int(segments[5])
                parent_id = int(segments[6])
                child_id  = int(segments[7])
                #print("link[%d](%d) begin" % (len(self.links), self_id))
                is_new = True
                for l in self.links:
                    if self_id == l.self_id:
                        is_new = False
                        break
                if is_new == True:
                    self.links.append(Link())
                    i = len(self.links) - 1
                    self.links[i].self_id   = int(segments[5])
                    self.links[i].parent_id = int(segments[6])
                    self.links[i].child_id  = int(segments[7])
                self.is_link = True
                index = 0
                for l in self.links:
                    if self_id == l.self_id:
                        link_no = index
                    index += 1
                    
            elif (num_seg==12 or num_seg==15) and segments[3]=='BEGIN' and segments[4]=='Joint':
                #print("num_seg = %d" % num_seg)
                #print("joint[%d] begin" % len(self.links))
                if segments[6] == 'parent':
                    #print("    parent begin")
                    self.is_parent_joint = True
                    self.tmp_joint.parent_id = int(segments[7])
                elif segments[6] == 'child':
                    #print("    child begin")
                    self.is_child_joint = True
                    self.tmp_joint.child_id = int(segments[7])
                    if num_seg == 15 and segments[12] == 'mimic':
                        #print("        child mimic")
                        self.tmp_mimic.is_mimic   = True
                        self.tmp_mimic.multiplier = float(segments[13])
                        self.tmp_mimic.offset     = float(segments[14])
                
                self.tmp_joint.id     = int(segments[5])
                self.tmp_joint.joint_type = segments[8]
                self.tmp_joint.axis.x = int(segments[9])
                self.tmp_joint.axis.y = int(segments[10])
                self.tmp_joint.axis.z = int(segments[11])
                
            elif num_seg==8 and segments[3]=='BEGIN' and segments[4]=='Joint' and segments[6]=='parent':
                self.is_parent_joint_part = True
                #print("        parent part begin")
                #print("is_parent_joint_part = True")
                
            elif num_seg==4 and segments[3]=='END':
                if self.is_link == True:
                    if self.is_parent_joint == True and self.is_child_joint == False:
                        if self.is_parent_joint_part == True:
                            self.is_parent_joint_part = False
                            #print("        parent part end")
                        else:
                            self.is_parent_joint = False
                            #print("    parent end")
                        
                    elif self.is_parent_joint == False and self.is_child_joint == True:
                        self.is_child_joint = False
                        self.tmp_mimic.is_mimic = False
                        #print("        child mimic end")
                        #print("    child end")
                        
                    else:
                        self.is_link = False
                        #print("Link end")
                        
            elif num_seg == 15 and segments[0] == str(1):
                if self.is_link == False:
                    print("Part: %s is NOT in a link!!" % segments[14])
                    print("Modify %s" % ldr_file)
                    return
                
                if link_no < 0:
                    print("Error: Link is NOT defined!!")
                    return
                
                self.links[link_no].parts.append(DaePart())
                part_no = len(self.links[link_no].parts) - 1

                if self.is_parent_joint_part == True:
                    self.links[link_no].parts[part_no].is_parent_joint = True
                    self.links[link_no].parts[part_no].joint = copy.deepcopy(self.tmp_joint)
                    
                if self.is_child_joint == True:
                    self.links[link_no].parts[part_no].is_child_joint = True
                    self.links[link_no].parts[part_no].joint = copy.deepcopy(self.tmp_joint)
                    if self.tmp_mimic.is_mimic == True:
                        self.links[link_no].parts[part_no].joint.mimic = copy.deepcopy(self.tmp_mimic)
                    
                color = segments[1]
                h_x = float(segments[2])     # x
                h_y = float(segments[3])     # y
                h_z = float(segments[4])     # z
                h_a = float(segments[5])     # a
                h_b = float(segments[6])     # b
                h_c = float(segments[7])     # c
                h_d = float(segments[8])     # d
                h_e = float(segments[9])     # e
                h_f = float(segments[10])    # f
                h_g = float(segments[11])    # g
                h_h = float(segments[12])    # h
                h_i = float(segments[13])    # i
                
                # H matrix of a part in ldraw
                ldrawHpart = numpy.array([
                    [h_a, h_b, h_c, h_x],
                    [h_d, h_e, h_f, h_y],
                    [h_g, h_h, h_i, h_z],
                    [0.0, 0.0, 0.0, 1.0]])
                ldrawHdae = numpy.dot(ldrawHpart, partHdae)
                # scaling from [LDU] to [m]
                ldrawHdae = numpy.dot(ldu2m, ldrawHdae)
                dae_file = segments[14].replace("dat", "dae")
                
                # H matrix of a dae part in ROS
                rosHdae = numpy.dot(rosHldraw, ldrawHdae)
                
                # rHd = rHl * lHd
                # lHd = (rHl)^(-1) * rHd
                
                self.links[link_no].parts[part_no].rosHdae = rosHdae
                self.links[link_no].parts[part_no].name = dae_file.replace(".dae", "")
                self.links[link_no].parts[part_no].ref_id = ref_id
                self.links[link_no].parts[part_no].ref_link = 'ref_' + str(ref_id) + '_link'
                
                rot_x = 0.0
                rot_y = 0.0
                rot_z = 0.0
                
                pos_x = 0.0
                pos_y = 0.0
                pos_z = 0.0
                
                scale = 1.0
                d = {
                    'refID' : ref_id,
                    'mesh' : "%s%s" % (g_dae_path, dae_file),
                    'bound_x' : 0.0,
                    'bound_y' : 0.0,
                    'bound_z' : 0.0,
                    'm_scale' :' %s' % str(scale),
                    'bound_roll' : 0.0,
                    'bound_pitch' : 0.0,
                    'bound_yaw' : 0.0,
                    'dim_x' : 0.0,
                    'dim_y' : 0.0,
                    'dim_z' : 0.0,
                    'pos_x': '%s' % str(pos_x),
                    'pos_y': '%s' % str(pos_y),
                    'pos_z': '%s' % str(pos_z),
                    'rot_x': '%s' % str(rot_x),
                    'rot_y': '%s' % str(rot_y),
                    'rot_z': '%s' % str(rot_z),
                }
                print(TP.link %d, file = fp_urdf)
                ref_id += 1
        ### load() end
        self.print()
        
        ### xml closing
        # print("</robot>", file = fp_urdf)
        fp_urdf.close()
        
    def detect_parent_ref_joints(self, id, parent_ref_link, child_ref_link, link, part, fp):
        if part.joint.joint_type == 'continuous' and part.joint.mimic.is_mimic == False:
            link.ref_joint = 'ref_' + str(id) + '_joint'
            print("link.self_id = %d, ref_joint = %s" % (link.self_id, link.ref_joint))

    def print_joints(self, id, parent_ref_link, child_ref_link, link, part, fp):
        #if part.joint.joint_type == 'continuous' and part.joint.mimic.is_mimic == True:
        if part.joint.joint_type == 'continuous':
            for l in self.links:
                for p in l.parts:
                    if p.ref_link == part.ref_link:
                        print("%s is included in link %d (%d, %d, %s)" % (p.ref_link, l.self_id, l.parent_id, l.child_id, l.child_joint_part.ref_link))
            a_x = part.joint.axis.x
            a_y = part.joint.axis.y
            a_z = part.joint.axis.z
            print("%d continous %d, %d, %d" % (part.joint.id, a_x, a_y, a_z))
        else:
            a_x = 0.0
            a_y = 0.0
            a_z = 0.0
            
        # rvizHdae is calculated in Lxf.py
        eul = TF.euler_from_matrix(part.linkHdae)
        origin_roll  = eul[0]
        origin_pitch = eul[1]
        origin_yaw   = eul[2]
        # origin_x = part.linkHdae[0, 3] * LDU2M
        # origin_y = part.linkHdae[1, 3] * LDU2M
        # origin_z = part.linkHdae[2, 3] * LDU2M
        origin_x = part.linkHdae[0, 3]
        origin_y = part.linkHdae[1, 3]
        origin_z = part.linkHdae[2, 3]
        
        if part.joint.joint_type == 'continuous' and part.joint.mimic.is_mimic == True:
            index = 0
            for ll in self.links:
                for pp in ll.parts:
                    if pp.is_parent_joint == True:
                        if part.joint.id == pp.joint.id:
                            print("child: %d (%s), parent: %d (%s), parent link: %d (%s)" % (p.ref_id, p.name, pp.ref_id, pp.name, ll.self_id, ll.root_part_ref_link))
                            joint_link_id = index
                index += 1
            d = {
                'joint_name' : self.links[joint_link_id].ref_joint,
                'gear_ratio' : '%s' % str(part.joint.mimic.multiplier),
                'offset'     : '%s' % str(part.joint.mimic.offset)
            }
            mimic = TP.mimic %d
        elif part.joint.joint_type == 'continuous' and part.joint.mimic.is_mimic == False:
            link.ref_joint = 'ref_' + str(id) + '_joint'
            print("link.self_id = %d, ref_joint = %s" % (link.self_id, link.ref_joint))
            mimic = ''
        else:
            mimic = ''
            
        d = {
            'refID' : id,
            'joint_type' : part.joint.joint_type,
            'parent_link' : parent_ref_link,
            'child_link' : child_ref_link,
            'origin_x' : '%s' % origin_x,
            'origin_y' : '%s'% origin_y,
            'origin_z' : '%s' % origin_z,
            'origin_roll' : '%s' % origin_roll,
            'origin_pitch' : '%s' % origin_pitch,
            'origin_yaw' : '%s' % origin_yaw,
            'axis_x' : a_x, 'axis_y' : a_y, 'axis_z' : a_z,
            'mimic' : '%s' % mimic
        }
        print(TP.joint %d, file = fp)

    def link_mode(self, link):
        mode = ''
        if link.self_id != link.parent_id and link.self_id != link.child_id:
            mode = 'middle'
            #print("%d: parent: True,  child: True,  middle" % link.self_id)
        elif link.self_id == link.parent_id and link.self_id != link.child_id:
            mode = 'root'
            #print("%d: parent: False, child: True,  root" % link.self_id)
        elif link.self_id != link.parent_id and link.self_id == link.child_id:
            mode = 'end'
            #print("%d: parent: True,  child: False, end" % link.self_id)
        else:
            print("Error: floating link???")
            mode = 'error'
            
        return mode
        
    def make_joints(self, urdf_file):
        fp_urdf = open(urdf_file, "a")

        # decide root part's ref id in each link 
        for l in self.links:
            for p in l.parts:
                if p.is_parent_joint == False and p.is_child_joint == False:
                    p.is_root = True
                    l.root_part_ref_link = p.ref_link
                    l.root_part = copy.deepcopy(p)
                    break
        for l in self.links:
            print("l.root_part_ref_link = %s" % l.root_part_ref_link)
                
        # detect parent and child links(parts) in a link
        for l in self.links:
            print("%d: num of parts = %s" % (l.self_id, len(l.parts)))
            for p in l.parts:
                if p.is_parent_joint == True:
                    print("    parent_joint = %d, %s" % (p.ref_id, p.name))
                    l.parent_joints.append(DaePart())
                    l.parent_joints[len(l.parent_joints)-1] = copy.deepcopy(p)
                if p.is_child_joint == True:
                    print("    child_joint  = %d, %s" % (p.ref_id, p.name))
                    l.child_joint_part = copy.deepcopy(p)
        for l in self.links:
            print("num of parent joints = %d" % len(l.parent_joints))
            for p in l.parent_joints:
                if p.is_parent_joint == True:
                    print('parent is correct, id = %d' % p.joint.parent_id)
                
        for l in self.links:
            mode = self.link_mode(l)
            for p in l.parts:
                if mode == 'root':
                    p.parent_ref_link = g_root
                else:
                    if p.is_child_joint == True:
                        for ll in self.links:
                            for pp in ll.parts:
                                if pp.is_parent_joint == True:
                                    if p.joint.id == pp.joint.id:
                                        print("child: %d (%s), parent: %d (%s), parent link: %d (%s)" % (p.ref_id, p.name, pp.ref_id, pp.name, ll.self_id, ll.root_part_ref_link))
                                        p.parent_ref_link = ll.root_part_ref_link
                    else:
                        p.parent_ref_link = l.root_part_ref_link
                        
        # rosHdae = rosHparent * parentHdae
        # <-> linkHdae = (rosHparent)^(-1) * rosHdae
        # calc relative H matrix related to the root in a link
        for l in self.links:
            mode = self.link_mode(l)

            # connect child link(part) to parent link(part)
            for p in l.parts:
                #print("p.parent_ref_link = %s" % p.parent_ref_link)
                if mode == 'root':
                    p.parent_ref_link = g_root
                    p.linkHdae = p.rosHdae
                    tmpH = numpy.identity(4)
                    #print("p.rosHparent * p.linkHdae = ")
                    #print(numpy.dot(tmpH, p.linkHdae))
                else:
                    if p.is_root == True:
                        for pp in l.parts:
                            if pp.is_child_joint == True:
                                p.parent_ref_link = pp.ref_link
                                p.linkHdae = numpy.dot(inverseHomogeneous(pp.rosHdae), p.rosHdae)
                                p.linkHdae = numpy.dot(self.ldu2m, p.linkHdae)
                                tmpH = numpy.identity(4)

                    elif p.is_child_joint == True:
                        for ll in self.links:
                            for pp in ll.parts:
                                if pp.is_parent_joint == True:
                                    if p.joint.id == pp.joint.id:
                                        #print("child: %d (%s), parent: %d (%s), parent link: %d (%s)" % (p.ref_id, p.name, pp.ref_id, pp.name, ll.self_id, ll.root_part_ref_link))
                                        p.parent_ref_link = ll.root_part_ref_link
                                        p.linkHdae = numpy.dot(inverseHomogeneous(ll.root_part.rosHdae), p.rosHdae)
                                        p.linkHdae = numpy.dot(self.ldu2m, p.linkHdae)
                                        tmpH = numpy.identity(4)
                    else:
                        p.parent_ref_link = l.root_part_ref_link
                        p.linkHdae = numpy.dot(inverseHomogeneous(l.root_part.rosHdae), p.rosHdae)
                        p.linkHdae = numpy.dot(self.ldu2m, p.linkHdae)
                        tmpH = l.root_part.rosHdae
                        
                
        ref_id = 0
        for l in self.links:
            for p in l.parts:
                parent_ref_link = p.parent_ref_link
                child_ref_link  = p.ref_link
                if parent_ref_link != child_ref_link:
                    self.detect_parent_ref_joints(ref_id, parent_ref_link, child_ref_link, l, p, fp_urdf)
                    ref_id += 1
                    
        ref_id = 0
        for l in self.links:
            for p in l.parts:
                parent_ref_link = p.parent_ref_link
                child_ref_link  = p.ref_link
                # print("parent_ref_link = %s" % parent_ref_link)
                # print("child_ref_link = %s" % child_ref_link)
                # print("ref_id = %d, id = %d" % (ref_id, p.ref_id))
                if parent_ref_link != child_ref_link:
                    self.print_joints(ref_id, parent_ref_link, child_ref_link, l, p, fp_urdf)
                    ref_id += 1
                    
        ### xml closing
        print("</robot>", file = fp_urdf)
        fp_urdf.close()



def main(argv, stdout, environ):
    progname = argv[0]
    # try:
    #     opts, args = getopt.getopt(sys.argv[1:], "h", ["help", "debug"])
    # except getopt.GetoptError as err:
    #     print(err, file=sys.stderr)
    #     usage(progname)
    #     sys.exit(2)
    print("progname = ", progname)
    # ldrFilePtr = open(argv[1], "r")
    # lines = ldrFilePtr.read().split('\n')
    # ldrFilePtr.close()
    # for line in lines:
    #     segments = line.split()
    #     print("segments =", segments)
    #     for seg in segments:
    #         print("seg =", seg)

    ldr_file = argv[1]
    ldr_file_body, ldr_file_ext = os.path.splitext(ldr_file)
    urdf_file = ldr_file_body + ".urdf"
    urdf = Urdf()
    urdf.load(ldr_file, urdf_file)
    urdf.make_joints(urdf_file)
        
if __name__ == "__main__":
    main(sys.argv, sys.stdout, os.environ)
