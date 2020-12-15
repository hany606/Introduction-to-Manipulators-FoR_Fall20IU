import numpy as np
import vpython as vp

class RobotVisualization_vpython:
    def __init__(self, rate=100, scale=1,radius={}, origin=[0.0, 0.0, 0.0], axe_length=0.2):
        self.rate = rate
        self.radius = {**{"link":0.005, "joint":0.006, "node":0.008, "axe":0.003, "trajectory_trail":0.004},
                       **radius}
        self.axe_color = vp.vector(1, 1, 1)
        self.link_color = vp.vector(242/255, 92/255, 25/255)
        self.joint_color = vp.vector(1,1,1)
        self.node_color = vp.vector(0,1,0)
        self.trail_color = vp.vector(1,0,0)
        self.text_color = vp.vector(0,1,0)
        self.origin = origin
        self.axe_length = axe_length
        self.axe_radius = 0.03*axe_length
        self.time_color = vp.vector(0,1,0)
        self.time_text_pos = vp.vector(-axe_length, axe_length,0)
        self.scale = scale

    def render_frame(self, frame_sub, axis=True):
        if(axis):
            frame = [["axe", self.origin, [1,0,0], self.axe_length],
                     ["axe", self.origin, [0,1,0], self.axe_length],
                     ["axe", self.origin, [0,0,1], self.axe_length]] + frame_sub
        else:
            frame = frame_sub
        c = []
        visible_idx = []
        for obj in frame:
            shape_type = obj[0]
            if(shape_type == "joint"):
                v = vp.vector(*obj[1])*self.scale
                color = self.joint_color if len(obj) <= 2 else vp.vector(*obj[2])
                c.append(vp.sphere(pos=v, radius=self.radius["joint"], color=color))
                # c.append(vp.box(pos=v, length=self.radius["joint"]*2, height=self.radius["joint"]*2, width=self.radius["joint"]*2, color=color))

            elif(shape_type == "node"):
                v = vp.vector(*obj[1])*self.scale
                color = self.node_color if len(obj) <= 2 else vp.vector(*obj[2])
                c.append(vp.sphere(pos=v, radius=self.radius["node"], color=color))
                # c.append(vp.box(pos=v, length=self.radius["node"]*2, height=self.radius["node"]*2, width=self.radius["node"]*2, color=color))
            elif(shape_type == "trajectory_trail"):
                visible_idx.append(len(c))
                v = vp.vector(*obj[1])*self.scale
                color = self.trail_color if len(obj) <= 2 else vp.vector(*obj[2])
                c.append(vp.sphere(pos=v, radius=self.radius["trajectory_trail"], color=color))
            elif(shape_type == "text_joint"):
                text = str(obj[1])
                v = vp.vector(*obj[2])*self.scale
                c.append(vp.text(text=text, pos=v,color=self.text_color, height=0.03))
            elif(shape_type == "time"):
                # text= "Steps: {:}, Seconds: {:.3f}".format(obj[1], obj[2])
                # text= "Steps: {:}".format(obj[1], obj[2])
                text= "Steps: {:}".format(obj[1])
                c.append(vp.label(text=text, pos=self.time_text_pos, align="right", color=self.time_color))
                # c.append(vp.text(text=text, pos=self.time_text_pos, align="right", height=self.time_text_height, billboard=True, emissive=True, color=self.time_color))
            else:   # link, axe
                v1 = vp.vector(*obj[1])*self.scale
                v2 = vp.vector(*obj[2])*self.scale

                if shape_type == "link":
                    r = self.radius["link"]   #0.011
                    _color = self.link_color

                elif shape_type == "axe":
                    r = self.radius["axe"]
                    _color = self.axe_color
                    c.append(vp.arrow(pos=v1,axis=v2, length=obj[3], color=_color, shaftwidth=self.axe_radius))
                    continue
                c.append(vp.curve(pos=[v1, v2], color=_color, radius=r))
        vp.rate(self.rate)
        for i in range(len(frame)):
            if(i in visible_idx):
                continue
            c[i].visible = False

if __name__ == "__main__":
    vis = RobotVisualization_vpython(rate=10, scale=0.0005)
    # frame = []
    
    # frame = [["link",[0,0,0],[0.3,0.1,0.1]], ["joint",[0.2,0.2,0.2]], ["node", [0.4,0.4,0.4]]]
    # frame += [["time", 0, 0]]
    frame = [
            ['link', [0, 0, 0], [0., 0., 0.]], 
            # ['link', [0., 0., 0.], [ 25.,   0., 400.]],
            ['link', [0., 0., 0.], [0,   0., 400.]],
            ['link', [0., 0., 400.], [25,   0., 400.]],
            ['link', [ 25.,   0., 400.], [585.,   0., 400.]], 
            ['link', [585.,   0., 400.], [610.,   0., 400.]], 
            ['link', [610.,   0., 400.], [1125.,    0.,  400.]], 
            ['link', [1125.,    0.,  400.], [1125.,    0.,  400.]], 
            ['link', [1125.,    0.,  400.], [1215.,    0.,  400.]], 
            ['link', [1215.,    0.,  400.], [1215.,    0.,  400.]], 
            ['joint', [0., 0., 0.]], 
            ['joint', [ 25.,   0., 400.]], 
            ['joint', [585.,   0., 400.]], 
            ['joint', [610.,   0., 400.]], 
            ['joint', [1125.,    0.,  400.]], 
            ['joint', [1125.,    0.,  400.]], 
            ['joint', [1215.,    0.,  400.]], 
            ['joint', [1215.,    0.,  400.]], 
            ['node', [1215.,    0.,  400.]], 
            ]

    while True:
        # for i in range(500):
        #     frame[-1][1] = i
        vis.render_frame(frame)

