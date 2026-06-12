from sensor_msgs.msg import LaserScan


class LaserReadings:
    def __init__(self, index, reading):
        self.index = index
        self.reading = reading


class LaserManager:
    def __init__(self):
        pass

    def read_front_laser(self, msg: LaserScan) -> float:
        if not msg.ranges:
            raise RuntimeError("No laser data received yet")

        middle_index = round(len(msg.ranges) / 2)
        return msg.ranges[middle_index]

    def find_angle_from_laser_reading(self, msg: LaserScan, index: int) -> float:
        angle = msg.angle_min + (index * msg.angle_increment)
        return angle

    def cluster_laser_data(self, readings: list) -> list:
        clusters = []
        current_cluster = []
        INTENSITY_THRESHOLD = 3850.0

        for index, reading_value in enumerate(readings):
            if reading_value > INTENSITY_THRESHOLD:
                if not current_cluster or index == current_cluster[-1].index + 1:
                    reading = LaserReadings(index, reading_value)
                    current_cluster.append(reading)
                else:
                    clusters.append(current_cluster)
                    current_cluster = []
                    reading = LaserReadings(index, reading_value)
                    current_cluster.append(reading)

        if current_cluster:
            clusters.append(current_cluster)

        return clusters