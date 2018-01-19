#!/usr/bin/env python



class CollisionWorldManager:

    def __init__(self, collision_world):
        self._world = collision_world
        self.collision_objects = {}

    def remove(self, to_remove):
        del self.collision_objects
        self._world.remove(self.collision_objects[to_remove])


    def add_table(self, table):
        self._world.remove_all()
        rot = tf.transformations.euler_from_quaternion([table.pose.orientation.x,
                                                        table.pose.orientation.y,
                                                        table.pose.orientation.z,
                                                        table.pose.orientation.w])
        self.collision_objects["table"] = self._world.add_box(x=table.scale.x, y=table.scale.y, z=table.pose.position.z + 0.02,
                                                             pose=geometry.pose(x=table.pose.position.x,
                                                                                y=table.pose.position.y,
                                                                                z=table.pose.position.z / 2 + 0.01,
                                                                                ei=rot[0], ej=rot[1], ek=rot[2]),
                                                             frame_id='map')

    def add_table_object(self, object, objectname):

        rot = tf.transformations.euler_from_quaternion([object.bbox.pose.orientation.x,
                                                        object.bbox.pose.orientation.y,
                                                        object.bbox.pose.orientation.z,
                                                        object.bbox.pose.orientation.w])
        self.collision_objects[objectname] = self._world.add_box(x=object.width * 0.8, y=object.length * 0.8,
                                                         z=object.height,
                                                         pose=geometry.pose(x=object.x, y=object.y,
                                                                            z=object.z, ei=rot[0], ej=rot[1],
                                                                            ek=rot[2]), frame_id='table')
        par_frame = 'table'
        child_frame = objectname
        return par_frame, child_frame, object.bbox.pose

    def add_cupboard(self, shelves):

        rand_index = random.randint(0, len(shelves) - 1)
        selected_shelf = shelves[rand_index]
        shelf_width = selected_shelf.scale.x
        shelf_length = selected_shelf.scale.y
        shelf_frame = 'shelf' + str(selected_shelf)
        rot = tf.transformations.euler_from_quaternion([selected_shelf.pose.orientation.x,
                                                        selected_shelf.pose.orientation.y,
                                                        selected_shelf.pose.orientation.z,
                                                        selected_shelf.pose.orientation.w])
        for shelf in shelves:
            self._world.add_box(x=shelf.scale.x, y=shelf.scale.y, z=shelf.scale.z + 0.02,
                                    pose=geometry.pose(x=shelf.pose.position.x, y=shelf.pose.position.y,
                                                       z=shelf.pose.position.z, ei=rot[0], ej=rot[1], ek=rot[2]),
                                    frame_id='map')

        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = shelf_frame
        if shelf_width > shelf_length:
            x = 0.05
            y = shelf_length
            self._world.add_box(x=x, y=y, z=1.8, pose=geometry.pose(x=-shelf_width / 2, y=0, z=0.9 - selected_shelf.pose.position.z), frame_id=shelf_frame)
            self._world.add_box(x=x, y=y, z=1.8, pose=geometry.pose(x=shelf_width / 2, y=0, z=0.9 - selected_shelf.pose.position.z), frame_id=shelf_frame)
        else:
            x = shelf_width
            y = 0.05
            self._world.add_box(x=x, y=y, z=1.8, pose=geometry.pose(x=0, y=-shelf_length / 2, z=0.9 - selected_shelf.pose.position.z), frame_id=shelf_frame)
            self._world.add_box(x=x, y=y, z=1.8, pose=geometry.pose(x=0, y=shelf_length / 2, z=0.9 - selected_shelf.pose.position.z), frame_id=shelf_frame)

    def add_selected_shelf(self,  selected_shelf, shelf_width, shelf_length):
        pass
