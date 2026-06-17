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

    def cluster_laser_data(self, readings: list, max_gap: int = 3) -> list:
        """
        Group laser intensity readings into clusters, tolerating dropped readings.

        Readings within INTENSITY_THRESHOLD/INTENSITY_MAX are considered valid hits.
        A new reading is added to the current cluster if it falls within `max_gap`
        indices of the last reading in that cluster (allowing for dropped/missing
        readings in between). Otherwise, the current cluster is closed out and a
        new one is started.

        Args:
            readings: list of intensity values, indexed by position.
            max_gap: maximum allowed index gap between consecutive readings in a
                cluster before it's considered a break (default 2, i.e. up to
                one dropped reading in between).
        """
        clusters = []
        current_cluster = []
        INTENSITY_THRESHOLD = 4200
        INTENSITY_MAX = 5050

        for index, reading_value in enumerate(readings):
            if INTENSITY_THRESHOLD < reading_value < INTENSITY_MAX:
                reading = LaserReadings(index, reading_value)

                if current_cluster and index - current_cluster[-1].index > max_gap:
                    clusters.append(current_cluster)
                    current_cluster = []

                current_cluster.append(reading)

        if current_cluster:
            clusters.append(current_cluster)

        return clusters