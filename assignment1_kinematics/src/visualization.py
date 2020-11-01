import numpy as np
import vpython as vp

class RobotVisualization:
    def __init__(self, rate=100, radius={"link":0.005, "joint":0.006, "node":0.008, "axe":0.003}, origin=[0.0, 0.0, 0.0], axe_length=0.2):
        self.rate = rate
        self.radius = radius
        self.axe_color = vp.vector(1, 1, 1)
        self.link_color = vp.vector(1, 0.4, 0.4)
        self.joint_color = vp.vector(1,1,1)
        self.node_color = vp.vector(0,1,0)
        self.text_color = vp.vector(0,1,0)
        self.origin = origin
        self.axe_length = axe_length
        self.axe_radius = 0.03*axe_length
        self.time_color = vp.vector(0,1,0)
        self.time_text_pos = vp.vector(-axe_length, axe_length,0)

    def render_frame(self, frame_sub, axis=True):
        if(axis):
            frame = [["axe", self.origin, [1,0,0], self.axe_length],
                     ["axe", self.origin, [0,1,0], self.axe_length],
                     ["axe", self.origin, [0,0,1], self.axe_length]] + frame_sub
        else:
            frame = frame_sub
        c = []
        for obj in frame:
            shape_type = obj[0]
            if(shape_type == "joint"):
                v = vp.vector(*obj[1])
                color = self.joint_color if len(obj) <= 2 else vp.vector(*obj[2])
                c.append(vp.sphere(pos=v, radius=self.radius["joint"], color=color))
            elif(shape_type == "node"):
                v = vp.vector(*obj[1])
                color = self.node_color if len(obj) <= 2 else vp.vector(*obj[2])
                c.append(vp.sphere(pos=v, radius=self.radius["node"], color=color))
            elif(shape_type == "text_joint"):
                text = str(obj[1])
                v = vp.vector(*obj[2])
                c.append(vp.text(text=text, pos=v,color=self.text_color, height=0.03))
            elif(shape_type == "time"):
                text= "Steps: {:}, Seconds: {:.3f}".format(obj[1], obj[2])
                c.append(vp.label(text=text, pos=self.time_text_pos, align="right", color=self.time_color))
                # c.append(vp.text(text=text, pos=self.time_text_pos, align="right", height=self.time_text_height, billboard=True, emissive=True, color=self.time_color))
            else:   # link, axe
                v1 = vp.vector(*obj[1])
                v2 = vp.vector(*obj[2])

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
            c[i].visible = False

if __name__ == "__main__":
    vis = RobotVisualization()
    # frame = []
    
    frame = [["link",[0,0,0],[0.3,0.1,0.1]], ["joint",[0.2,0.2,0.2]], ["node", [0.4,0.4,0.4]]]
    frame += [["time", 0, 0]]
    while True:
        for i in range(500):
            frame[-1][1] = i
            vis.render_frame(frame)

