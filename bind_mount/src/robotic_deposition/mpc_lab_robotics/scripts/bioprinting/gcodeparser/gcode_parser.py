from typing import List, Dict, Tuple, Union
from dataclasses import dataclass
import re
from .commands import Commands
import numpy as np
import math


@dataclass
class GcodeLine:
    command: Tuple[str, int]
    params: Dict[str, float]
    comment: str

    def __post_init__(self):
        if self.command[0] == 'G' and self.command[1] in (0, 1, 2, 3):
            self.type = Commands.MOVE
        elif self.command[0] == ';':
            self.type = Commands.COMMENT
        elif self.command[0] == 'T':
            self.type = Commands.TOOLCHANGE
        else:
            self.type = Commands.OTHER

    @property
    def command_str(self):
        return f"{self.command[0]}{self.command[1] if self.command[1] is not None else ''}"

    def get_param(self, param: str, return_type=None, default=None):
        """
        Returns the value of the param if it exists, otherwise it will the default value.
        If `return_type` is set, the return value will be type cast.
        """
        try:
            if return_type is None:
                return self.params[param]
            else:
                return return_type(self.params[param])
        except KeyError:
            return default

    def update_param(self, param: str, value: Union[int, float]):
        if self.get_param(param) is None:
            return
        if type(value) not in (int, float):
            raise TypeError(f"Type {type(value)} is not a valid parameter type")
        self.params[param] = value
        return self.get_param(param)

    def delete_param(self, param: str):
        if self.get_param(param) is None:
            return
        self.params.pop(param)

    @property
    def gcode_str(self):
        command = self.command_str
        params = " ".join(f"{param}{self.get_param(param)}" for param in self.params.keys())
        comment = f"; {self.comment}" if self.comment != '' else ""
        if command == ';':
            return comment
        return f"{command} {params} {comment}".strip()

# @dataclass
# class PyCodeLine:
#     command: Tuple[str, int]
#     params: Dict[str, float]
#     comment: str

class GcodeParser:
    def __init__(self, gcode: str, include_comments=True):
        self.gcode = gcode
        self.lines: List[GcodeLine] = get_lines(self.gcode, include_comments)
        self.include_comments = include_comments
        self.layers = get_layers(self.lines)

    def get_layer_height(self):
        '''will not work, if the G-Code is not annotated using the respective Prusa Slicer option'''
        height = None
        for line in self.lines:
            if 'HEIGHT' in line.comment:
                height = float(re.findall(r"[-+]?(?:\d*\.\d+|\d+)", line.comment)[0])
                break
        return height

    def get_number_of_layers(self):
        layer_count = 0
        for line in self.lines:
            if 'next layer' in line.comment:
                layer_count += 1
        return layer_count

    def scale_model(self, factor):
        for line in self.lines:
            if 'X' in line.params:
                line.update_param('X', factor*line.get_param('X'))
            if 'Y' in line.params:
                line.update_param('Y', factor*line.get_param('Y'))
            if 'Z' in line.params:
                line.update_param('Z', factor*line.get_param('Z'))

    def remove_X_Y_offset(self):
        '''removes any X-Y plane offset to init pose, i.e. the printing start point is moved to 0,0
        '''
        # detect X,Y position offset to move printing start position to self.near_touch, find first printing location
        x_offset = 0
        y_offset = 0
        old_x = 0
        old_y = 0
        for line in self.lines:
            if 'X' in line.params:
                x_offset = line.get_param('X')
            if 'Y' in line.params:
                y_offset = line.get_param('Y')
            if x_offset != 0 and y_offset != 0:
                break

        #remove X,Y offset from coordinates
        for line in self.lines:
            if 'X' in line.params:
                old_x = line.get_param('X')
                line.update_param('X', old_x - x_offset)
            if 'Y' in line.params:
                old_x = line.get_param('Y')
                line.update_param('Y', old_x - x_offset)

    def get_first_mid_last_layer_moves(self,planar_only = False):
        #header [movement_count,'X','Y','Z','Comment','isExtruding', 'feedrate','layer_height',line_idx]
        #return first_layer_moves, mid_layer_moves_one, mid_layer_moves_one final_layer_moves, all_movements
        #where the mid_layer_moves stem from two consecutive layers (to allow alternating infill patterns)
        #@todo: check if planar_only = False: still working?
        last_x = 0
        last_y = 0
        last_z = 0
        current_feedrate = 0
        movement_lines = []
        comments = []
        output = []
        layer_changes = []
        first_layer_moves =[]
        extruding = False
        count = 0
        layer_height = 0
        flag_new_layer = False

        for index, line in enumerate(self.lines):
            if 'F' in line.params:
                current_feedrate = line.get_param('F')

            if line.comment == 'LAYER_CHANGE':
                flag_new_layer = True

            if any(param in line.params for param in {'X','Y'}) and any(command in line.command_str for command in {'G0','G1'}):
                if line.get_param('E',default=0) > 0:
                    extruding = True
                else:
                    extruding = False
                movement_lines.append(line)
                x = line.get_param('X', float, default = last_x)
                y = line.get_param('Y', float, default = last_y)
                z = line.get_param('Z', float, default = last_z)
                last_x = x
                last_y = y
                last_z = z
                if flag_new_layer == True:
                    comment = 'New Layer'
                else:
                    comment = ''
                output_line = {"output_number": count, "X": x,"Y": y,"Z": z,"comment": comment,"extruding": extruding, 'F': current_feedrate, 'layer_height': layer_height, "idx": index}
                output.append(output_line)
                flag_new_layer = False
                count += 1
            elif 'Z' in line.params:
                    if 'lift' not in line.comment and 'retract' not in line.comment and any(command in line.command_str for command in {'G0','G1'}): 
                        extruding = False
                        movement_lines.append(line)
                        x = line.get_param('X', float, default = last_x)
                        y = line.get_param('Y', float, default = last_y)
                        z = line.get_param('Z', float, default = last_z)
                        layer_height = z - last_z
                        last_x = x
                        last_y = y
                        last_z = z
                        if flag_new_layer == True:
                            comment = 'New Layer'
                        else:
                            comment = ''
                        if planar_only == False:
                            output_line = {"output_number": count, "X": x,"Y": y,"Z": z,"comment": comment, "extruding": extruding,'F': current_feedrate, 'layer_height': layer_height, "idx": index}
                            output.append(output_line)
                            flag_new_layer = False
                            count += 1

        for move in output:
            if move["comment"] != '':
                layer_changes.append(move)
        
        #first layer
        first_layer_moves = output[layer_changes[0]["output_number"]:layer_changes[1]["output_number"]]
        if planar_only == False: #get X-Y coordinates for first move
            first_layer_moves[0]['X'] = first_layer_moves[1]['X']
            first_layer_moves[0]['Y'] = first_layer_moves[1]['X']
        
        #mid layer
        mid_layer_moves_one = output[layer_changes[math.floor(len(layer_changes)/2)]["output_number"]:layer_changes[math.floor(len(layer_changes)/2)+1]["output_number"]]
        mid_layer_moves_two = output[layer_changes[math.floor(len(layer_changes)/2)+1]["output_number"]:layer_changes[math.floor(len(layer_changes)/2)+2]["output_number"]]

        #final Layer
        final_layer_moves = output[layer_changes[-1]["output_number"]:]

        return first_layer_moves, mid_layer_moves_one, mid_layer_moves_two, final_layer_moves, output


def get_layers(lines):
    #creates an own list of all movement lines for each layer respectively
    layer_startidx = []
    idx = 0
    layers = []

    #find line where new layer starts
    for line in lines:
        if 'LAYER_CHANGE' in line.comment:
            layer_startidx.append(idx)
        idx += 1

    #separate movement commands to individual lists for each layer
    for i in range(len(layer_startidx)):
        layercode: List[GcodeLine] = []
        if i != len(layer_startidx)-1:
            for line in lines[layer_startidx[i]:layer_startidx[i+1]]:
                if line.command_str == 'G0' or line.command_str =='G1':
                    layercode.append(line)
        #last layer
        else:
            for line in lines[layer_startidx[i]:]:
                if line.command_str == 'G0' or line.command_str =='G1':
                    layercode.append(line)
        layers.append(layercode)
    return layers

 
def get_lines(gcode, include_comments=False):
    regex = r'(?!; *.+)(G|M|T|g|m|t)(\d+)(([ \t]*(?!G|M|g|m)\w(".*"|([-\d\.]*)))*)[ \t]*(;[ \t]*(.*))?|;[ \t]*(.+)'
    regex_lines = re.findall(regex, gcode)
    lines = []
    for line in regex_lines:
        if line[0]:
            command = (line[0].upper(), int(line[1]))
            comment = line[-2]
            params = split_params(line[2])

        elif include_comments:
            command = (';', None)
            comment = line[-1]
            params = {}

        else:
            continue

        lines.append(
            GcodeLine(
                command=command,
                params=params,
                comment=comment.strip(),
            ))

    return lines

def element_type(element: str):
    if re.search(r'"', element):
        return str
    if re.search(r'\..*\.', element):
        return str
    if re.search(r'[+-]?\d*\.', element):
        return float
    return int


def split_params(line):
    regex = r'((?!\d)\w+?)(".*"|(\d+\.?)+|-?\d*\.?\d*)'
    elements = re.findall(regex, line)
    params = {}
    for element in elements:
        try:
            params[element[0].upper()] = element_type(element[1])(element[1])
        except:
            print('Line skipped')

    return params


if __name__ == '__main__':
    with open('3DBenchy.gcode', 'r') as f:
        gcode = f.read()
    parsed_gcode = GcodeParser(gcode)
    pass
