import pybullet as p
from modules.module import Module
from config import ROBOT_URDF_PATH, width, circumcircle_radius

class Robot:
    def __init__(self, module_path, configuration):

       module_paths = module_path * 3 
       self.modules = []
       # Load first module at origin
       self.modules.append(Module(module_paths[0]))

       body_id = self.modules[0].robot_id  # or whichever module you want
       num_joints = p.getNumJoints(body_id)
       for i in range(num_joints):
          info = p.getJointInfo(body_id, i)
          print(f"Link index: {i}, Linkname: {info[12].decode('utf-8')}")



       # Load additional modules based on configuration
       if configuration == "line":
            # Face 3 is left, face 0 is right (assuming face 0 is to the right of origin)
            # Determine offsets for left and right modules
            left_offset = hex_face_offset(3, width)
            # Determine positions for left and right modules
            # Use the first module's start position as the base
            left_pos = [self.modules[0].start_pos[0] + left_offset[0], self.modules[0].start_pos[1] + left_offset[1], self.modules[0].start_pos[2]]
           
            
       elif configuration == "arrow":
            #todo
            NotImplemented
       else:
           raise ValueError("Unknown configuration: {}".format(configuration))

       # Load left and right modules
       # Use the same orientation as the first module
       self.modules.append(Module(module_paths[1], start_pos=left_pos, start_euler=self.modules[0].start_euler))
    
       if configuration == "line":
          # Connect link_1 of module 0 to link_2 of module 1 with a fixed constraint
          body_a = self.modules[0].robot_id
          body_b = self.modules[1].robot_id
          link_a = 1  # connector_link_1
          link_b = 2  # connector_link_2

          cid = p.createConstraint(
          parentBodyUniqueId=body_a,
          parentLinkIndex=link_a,
          childBodyUniqueId=body_b,
          childLinkIndex=link_b,
          jointType=p.JOINT_FIXED,
          jointAxis=[0, 0, 0],
          parentFramePosition=[0, 0, 0],
          childFramePosition=[0, 0, 0]
          ) 
     
       elif configuration == "arrow":
            #todo
            NotImplemented
       else:
           raise ValueError("Unknown configuration: {}".format(configuration))

# helper function determines offset so that faces match between modules
# this is used to align modules when they are connected
import numpy as np
def hex_face_offset(face_index, radius=circumcircle_radius):
    """Return local position of a hex face center (0-5)."""
    angle_rad = np.radians(face_index * 60 + 30) #pointy end at the top
    x = radius * np.cos(angle_rad)
    y = radius * np.sin(angle_rad)
    return [x, y, 0]import pybullet as p
from modules.module import Module
from config import ROBOT_URDF_PATH, width, circumcircle_radius

class Robot:
    def __init__(self, module_path, configuration):

       module_paths = module_path * 3 
       self.modules = []
       # Load first module at origin
       self.modules.append(Module(module_paths[0]))

       body_id = self.modules[0].robot_id  # or whichever module you want
       num_joints = p.getNumJoints(body_id)
       for i in range(num_joints):
          info = p.getJointInfo(body_id, i)
          print(f"Link index: {i}, Linkname: {info[12].decode('utf-8')}")



       # Load additional modules based on configuration
       if configuration == "line":
            # Face 3 is left, face 0 is right (assuming face 0 is to the right of origin)
            # Determine offsets for left and right modules
            left_offset = hex_face_offset(3, width)
            right_offset = hex_face_offset(0, width) 
            # Determine positions for left and right modules
            # Use the first module's start position as the base
            left_pos = [self.modules[0].start_pos[0] + left_offset[0], self.modules[0].start_pos[1] + left_offset[1], self.modules[0].start_pos[2]]
            right_pos = [self.modules[0].start_pos[0] + right_offset[0], self.modules[0].start_pos[1] + right_offset[1], self.modules[0].start_pos[2]]
            
       elif configuration == "arrow":
            #todo
            NotImplemented
       else:
           raise ValueError("Unknown configuration: {}".format(configuration))

       # Load left and right modules
       # Use the same orientation as the first module
       self.modules.append(Module(module_paths[1], start_pos=left_pos, start_euler=self.modules[0].start_euler))
       self.modules.append(Module(module_paths[2], start_pos=right_pos, start_euler=self.modules[0].start_euler))
    
       if configuration == "line":
          # Connect link_1 of module 0 to link_2 of module 1 with a fixed constraint
          body_a = self.modules[0].robot_id
          body_b = self.modules[1].robot_id
          link_a = 1  # connector_link_1
          link_b = 2  # connector_link_2

          cid = p.createConstraint(
          parentBodyUniqueId=body_a,
          parentLinkIndex=link_a,
          childBodyUniqueId=body_b,
          childLinkIndex=link_b,
          jointType=p.JOINT_FIXED,
          jointAxis=[0, 0, 0],
          parentFramePosition=[0, 0, 0],
          childFramePosition=[0, 0, 0]
          ) 
          # Connect link_2 of module 0 to link_1 of module 2 with a fixed constraint
          body_a = self.modules[0].robot_id
          body_b = self.modules[2].robot_id
          link_a = 2  # connector_link_1
          link_b = 1  # connector_link_2

          cid = p.createConstraint(
          parentBodyUniqueId=body_a,
          parentLinkIndex=link_a,
          childBodyUniqueId=body_b,
          childLinkIndex=link_b,
          jointType=p.JOINT_FIXED,
          jointAxis=[0, 0, 0],
          parentFramePosition=[0, 0, 0],
          childFramePosition=[0, 0, 0]
          ) 
       elif configuration == "arrow":
            #todo
            NotImplemented
       else:
           raise ValueError("Unknown configuration: {}".format(configuration))

# helper function determines offset so that faces match between modules
# this is used to align modules when they are connected
import numpy as np
def hex_face_offset(face_index, radius=circumcircle_radius):
    """Return local position of a hex face center (0-5)."""
    angle_rad = np.radians(face_index * 60 + 30) #pointy end at the top
    x = radius * np.cos(angle_rad)
    y = radius * np.sin(angle_rad)
    return [x, y, 0]
