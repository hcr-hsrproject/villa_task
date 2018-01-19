import random

import operator


class WorldModel:
    def __init__(self):
        self.shelves = None
        self.table = None
        self.objects = None
        self.current_object = None

    def add_table(self, table):
        self.table = table

    def add_shelves(self, shelves):
        self.shelves = shelves

    def add_table_objects(self, objects):
        self.objects = objects

    def sort_objects(self, axis):
        obj_dict = {}
        if axis == "x":
            for obj in self.objects:
                obj_dict[obj] = self.objects[obj].length
        else:
            for obj in self.objects:
                obj_dict[obj] = self.objects[obj].width
        sorted_objects = sorted(obj_dict.items(), key=operator.itemgetter(1))
        return sorted_objects

    def set_axis(self, axis):
        self.axis = axis

    def picked_up_object(self, object_name):
        self.current_object = self.objects[object_name]
        del self.objects[object_name]