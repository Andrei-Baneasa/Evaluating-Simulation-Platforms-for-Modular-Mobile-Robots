import pybullet as p
import pybullet_data

def setup_environment():
    # Configure PyBullet #
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81) # set Earth gravity

    #Create ground using plane shape
    ground_collision = p.createCollisionShape(p.GEOM_PLANE)
    ground_visual = p.createVisualShape(p.GEOM_PLANE, rgbaColor=[0.3, 0.8, 0.3, 1])
    ground_id = p.createMultiBody(baseMass=0,
                             baseCollisionShapeIndex=ground_collision,
                             baseVisualShapeIndex=ground_visual,
                             basePosition=[0, 0, 0])