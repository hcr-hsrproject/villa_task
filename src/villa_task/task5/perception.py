import random
from villa_manipulation import bwi_perception_module

class PerceptionManager:
    def __init__(self):
        self.plane_object_segmentation = bwi_perception_module.BWISegmentation()
        pass

    def select_shelf(self, world_model):
        index = random.randint(0, len(world_model.shelves) - 1)
        return world_model.shelves[index], "shelf" + str(index)
    
    def decide_placement(self, shelf, shelf_objects, object_in_hand, inverse_transform):
        # Put object along major axis, a quarter of the way onto the shelf
        adjustment_factor = max(object_in_hand.length, object_in_hand.width) + 0.08

        shelf_width = shelf.scale.x
        shelf_length = shelf.scale.y
        if shelf_width > shelf_length:  # put along x axis
            x = - shelf_width / 2 + (shelf_width / 5) * random.randint(1, 4)
            if inverse_transform.transform.translation.y > 0:
                y = shelf_length / 4 + adjustment_factor
            else:
                y = - shelf_length / 4 - adjustment_factor
        else:
            if inverse_transform.transform.translation.x > 0:
                x = shelf_width / 4 + adjustment_factor
            else:
                x = -shelf_width / 4 - adjustment_factor
            y = - shelf_length / 2 + (shelf_width / 5) * random.randint(1, 4)
        print "2D placement pose:", x, ',', y
        return x, y
