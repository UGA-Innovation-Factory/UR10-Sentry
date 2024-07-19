import time
import math


class BBoxProcessor:

    def __init__(self):
        self.last_boxes = []
        self.last_timestamps = []

    def get_joystick_position_from_new_set_of_bboxes(self, new_boxes: list) -> list:
        """
        Given a set of bounding boxes, returns the joystick position that would move the robot to the closest one.
        """
        new_boxes = self.process_next(new_boxes)
        closest_box = self.get_box_closest_to_center(new_boxes)
        if closest_box is None:
            return [0, 0]
        print("Closest box: ", closest_box, "All: ", new_boxes)
        return self.get_normalized_box_position(closest_box)

    def bbox_distance(self, box1: list, box2: list) -> float:
        """
        Calculates the distance between two bounding boxes.
        """
        return math.sqrt(
            ((box1[0] + box1[2] / 2) - (box2[0] + box2[2] / 2)) ** 2
            + ((box1[1] + box1[3] / 2) - (box2[1] + box2[3] / 2)) ** 2
        )

    def process_next(self, new_boxes, dist_threshold=80) -> list:
        """
        Receives a reading of bounding boxes and processes them.
        It compares previous bounding boxes with the new ones: If a previous box is close enough to an new one, it is considered the same object.
        It also checks if previous boxes have outlived their last appearance, and if so, they are not added to the new list.
        """
        timestamp = time.time()
        time_to_live = 0.5

        new_boxes_list = []
        new_timestamps_list = []
        for new_box in new_boxes:
            new_boxes_list.append(new_box)
            new_timestamps_list.append(timestamp)

        min_distance = dist_threshold
        for index, box in enumerate(self.last_boxes):
            distance = 9999
            closest = None
            for new_box in new_boxes:
                new_distance = self.bbox_distance(box, new_box)
                if new_distance < min_distance and new_distance < distance:
                    distance = new_distance
                    closest = new_box

            if (
                closest is None
                and timestamp < self.last_timestamps[index] + time_to_live
            ):
                new_boxes_list.append(box)
                new_timestamps_list.append(timestamp)

        self.last_boxes = new_boxes_list
        self.last_timestamps = new_timestamps_list
        return new_boxes_list

    def get_box_closest_to_center(self, box_list: list) -> list:
        """
        Returns the BBox closest to the center (500, 500)
        """
        closest = []
        closest_distance = 9999
        for box in box_list:
            distance = math.sqrt(
                (box[0] + box[2] / 2 - 500) ** 2 + (box[1] + box[3] / 2 - 500) ** 2
            )
            if distance < closest_distance:
                closest = box
                closest_distance = distance
        return closest

    def get_normalized_box_position(self, box: list) -> list:
        """
        Returns the normalized position of a bounding box, shringking its range from 0 to 1000 to 0 to 1
        """
        return [round((box[0] + box[2] / 2) / 500.0 - 1, 3), round((box[1] + box[3] / 2) / 500 - 1, 3)]
