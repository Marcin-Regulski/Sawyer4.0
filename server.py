from flask import Flask, render_template
from flask import request

import argparse
import struct
import sys
import copy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

app = Flask(__name__)

part1 = ''
part2 = ''

#all the poses and joint angles
class Sequence(object):
    def __init__(self):

        self._assembly_sequence = []
        self._test_sequence = []


        self._parts_location = "/home/nitro/sawyer_ws/src/sawyer_gripper/src/parts.txt"

        self._overhead_orientation = Quaternion(
                             x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936)

        self._flat_orientation = Quaternion(
                             x=0.7,
                             y=-0.7,
                             z=0.0,
                             w=0.0)

        self._flat2_orientation = Quaternion(
                             x=0.7,
                             y=0.0,
                             z=0.7,
                             w=0.0)

        self._zero_pose=Pose(position=Point(x=0.589481865788,
                                           y=0.345138618853, 
                                           z= 0.330077121125),
                             orientation = self._overhead_orientation)

        self.poses = {
            #angles
            'Start_angles'  : [1.69, -0.06, -2.97, 1.44, -1.23, -1.38, 1.36],

            #starting pose, with only base to pick up (assembly)
            'Belt_pick_base'  :[Pose( position=Point(x=self._zero_pose.position.x +0.2255181342,
                                                     y=self._zero_pose.position.y -0.2221386189,
                                                     z=self._zero_pose.position.z -0.03607715113),
                                      orientation=self._flat2_orientation),
                              '0.05',
                              '0.028'],
            #ending pose, with only base to place down (assembly)
            'Belt_place_base' :[Pose( position=Point(x=self._zero_pose.position.x +0.2255181342,
                                                     y=self._zero_pose.position.y -0.2221386189,
                                                     z=self._zero_pose.position.z -0.03607715113),
                                      orientation=self._flat2_orientation),
                              '0.028',
                              '0.05'],

            #starting pose, with full product to pick up (testing)
            'Belt_pick_full'  :[Pose( position=Point(x=self._zero_pose.position.x +0.2255181342,
                                                     y=self._zero_pose.position.y -0.2221386189,
                                                     z=self._zero_pose.position.z -0.02407715113),
                                      orientation=self._flat2_orientation),
                              '0.05',
                              '0.028'],
            #ending pose, with full product to place down (testing)
            'Belt_place_full' :[Pose( position=Point(x=self._zero_pose.position.x +0.2255181342,
                                                     y=self._zero_pose.position.y -0.2221386189,
                                                     z=self._zero_pose.position.z -0.02607715113),
                                      orientation=self._flat2_orientation),
                              '0.028',
                              '0.05'],                  
            #ending pose, with only base to place down (assembly)                  
            'Clamp_pick_base' :[Pose( position=Point(x=self._zero_pose.position.x -0.05297288865,
                                                y=self._zero_pose.position.y +0.00320372292,
                                                z=self._zero_pose.position.z -0.09131049364),
                                    orientation = self._flat2_orientation),
                              '0.05',
                              '0.028'],
            #starting pose, with only base to place down (assembly)                   
            'Clamp_place_base' :[Pose( position=Point(x=self._zero_pose.position.x -0.05297288865,
                                                y=self._zero_pose.position.y +0.00355372292,
                                                z=self._zero_pose.position.z -0.09131049364),
                                    orientation = self._flat2_orientation),
                              '0.028',
                              '0.05'],
            #ending pose, with full product to place down (testing) 
            'Clamp_pick_full' :[Pose( position=Point(x=self._zero_pose.position.x -0.05297288865,
                                                y=self._zero_pose.position.y +0.00320372292,
                                                z=self._zero_pose.position.z -0.08131049364),
                                    orientation = self._flat2_orientation),
                              '0.05',
                              '0.028'],
            #starting pose, with full product to place down (testing) 
            'Clamp_place_full' :[Pose( position=Point(x=self._zero_pose.position.x -0.05297288865,
                                                y=self._zero_pose.position.y +0.00355372292,
                                                z=self._zero_pose.position.z -0.08131049364),
                                    orientation = self._flat2_orientation),
                              '0.028',
                              '0.05'],

            'PCB'              :[Pose( position=Point(x=self._zero_pose.position.x -0.06613,
                                                y=self._zero_pose.position.y +0.126525,
                                                z=self._zero_pose.position.z +0.0073),
                                    orientation = self._flat_orientation),
                               '0',
                               '0.02'],

            'Clamp_PCB'        :[Pose( position=Point(x=self._zero_pose.position.x +0.03138,
                                                y=self._zero_pose.position.y,
                                                z=self._zero_pose.position.z +0.0107),
                                    orientation = self._overhead_orientation),
                               '0.02',
                               '0'],

            'Fuse'             :[Pose(position = Point(x=self._zero_pose.position.x +0.03188,
                                            y=self._zero_pose.position.y +0.108525,
                                            z=self._zero_pose.position.z +0.01212),
                                    orientation = self._overhead_orientation),
                               '0.01',
                               '0'],

            'Clamp_Fuse'       :[Pose( position=Point(x=self._zero_pose.position.x +0.03038,
                                                y=self._zero_pose.position.y +0.04308,
                                                z=self._zero_pose.position.z +0.01540),
                                    orientation = self._overhead_orientation),
                               '0',
                               '0.012'],

            'Camera_check'     :[Pose( position=Point(x=self._zero_pose.position.x +0.127427111,
                                                y=self._zero_pose.position.y +0.04105372292,
                                                z=self._zero_pose.position.z -0.07931049364+0.1),
                                    orientation = self._flat2_orientation),
                               '0',
                               '0'],

            'Button_press'     :[Pose( position=Point(x=self._zero_pose.position.x +0.02838,
                                                y=self._zero_pose.position.y + 0.0243,
                                                z=self._zero_pose.position.z +0.0257),
                                    orientation = self._flat_orientation),
                               '0',
                               '0'],
            
            #angles 
            'Camera_pos'    : [-1.44541015625, 
                               -0.5413984375, 
                                0.5386044921875,
                                1.349265625, 
                               -0.53997265625,
                               -0.6926015625, 
                                0.6488369140625],
            #angles 
            'Midpoint'      : [-0.81357421875,
                               -0.230709960938,
                               -0.194439453125,
                                1.56727148437,
                               -1.07634277344,
                               -1.13403417969,
                                1.01579589844],
            'Error'         : '-----------SOMETHING WENT WRONG-----------'
            }

    def _create_sequence(self,stage):
        sequence = []
        if stage == "assembly":

            beg_list = ['Start_angles','Belt_pick_base','Midpoint','Clamp_place_base']
            for i in beg_list:
                sequence.append(self.poses[i])
        
            try: 
                with open(self._parts_location, "r") as f:
                    content = f.readlines()
                    content = [x.strip() for x in content]
                f.close()
            except:
                content = []

            if content != []:
                for j in content:
                    sequence.append(self.poses[j])
                    if j == "PCB":
                        sequence.append(self.poses["Clamp_PCB"])
                    elif j == "Fuse":
                        sequence.append(self.poses["Clamp_Fuse"])
                    elif j == "Error":
                        sequence.append("ERRORO")
                         
            beg_list = ['Clamp_pick_base','Belt_place_base','Start_angles']
            for k in beg_list:
                sequence.append(self.poses[k])

        elif stage == "testing":
            testing_list = ['Start_angles','Belt_pick_full','Midpoint','Clamp_place_full','Button_press',
                            'Midpoint','Camera_pos','Camera_check','Clamp_pick_full','Belt_place_full','Start_angles']
            for l in testing_list:
                sequence.append(self.poses[l])

        return sequence

    def request_pose(self,stage,number):
        sequence = self._create_sequence(stage)
        #returns a list
        return sequence[number]


#default index page
@app.route('/')
def index():
    #note that template loads for all pages after slash
    return render_template('index.html')

#page for 1st stage, assembly
@app.route('/assembly')
def assembly():
    if request.args.get('part1') is not None:
        part1 = request.args.get('part1')
        part2 = request.args.get('part2')
        #part3 = request.args.get('part3')
        #part4 = request.args.get('part4')
        #part5 = request.args.get('part5')'''
        f = open("/home/nitro/sawyer_ws/src/sawyer_gripper/src/parts.txt","w+")
        f.write(part1+"\n"+part2)
        f.close()
        return 'parts 1 is: '+ part1 + ', part2 is: ' + part2
    #replace with template if no parts have been chosen
    else: return "no parts chosen. Reload"

#page for 2nd stage, testing
@app.route('/testing')
def testing():
    # works! ;D
    cp = Sequence()
    listy = cp._create_sequence("assembly")
    return str(len(listy))+str(listy)
    #return cp.poses["Fuse"].

    #return "dafuq"
    #new_list = cp._get_beggining_of_sequence()
    #return str(new_list[1].orientation)





if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
