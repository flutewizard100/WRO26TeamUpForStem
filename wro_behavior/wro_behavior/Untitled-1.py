class Hypothesis:
    def __init__(self, class_id):
        self.class_id = class_id


class Position:
    def __init__(self, z):
        self.z = z


class PoseData:
    def __init__(self, z):
        self.position = Position(z)


class PoseWrapper:
    def __init__(self, z):
        self.pose = PoseData(z)


class Result:
    def __init__(self, class_id, z):
        self.hypothesis = Hypothesis(class_id)
        self.pose = PoseWrapper(z)


class Detection:
    def __init__(self, class_id, z):
        self.results = [Result(class_id, z)]


class Scan:
    def __init__(self, ranges, angle_min, angle_increment):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment


THETA_1 = 1.0
DIP_THRESHOLD = 0.5


class TestRobot:
    def __init__(self):
        self.detections = []
        self.latest_scan = Scan(
            ranges=[2.0] * 20,
            angle_min=-1.57,
            angle_increment=0.1
        )
        self.blockPositionsArray = []
        self.OFFSET_ANGLE_NUM = 3
        self.test_angle_distances = {}

    def range_at_angle(self, angle):
        return self.test_angle_distances.get(angle, 3.0)

    def set_fake_range_section(self, section_number):
        """
        section_number:
            1 -> dip in i=0
            2 -> dip in i=1
            3 -> dip in i=2
        """
        self.test_angle_distances = {}

        # default all distances flat
        for i in range(3):
            for j in range(self.OFFSET_ANGLE_NUM):
                angle = i * THETA_1 + j * (THETA_1 / self.OFFSET_ANGLE_NUM)
                self.test_angle_distances[angle] = 3.0

        # put a dip in the chosen section
        target_i = section_number - 1
        if target_i in [0, 1, 2]:
            angle0 = target_i * THETA_1 + 0 * (THETA_1 / self.OFFSET_ANGLE_NUM)
            angle1 = target_i * THETA_1 + 1 * (THETA_1 / self.OFFSET_ANGLE_NUM)

            self.test_angle_distances[angle0] = 3.0
            self.test_angle_distances[angle1] = 2.0  # jump of 1.0 > DIP_THRESHOLD

    def addBlocks(self):
        currentSectionArray = []
        NEAR = 1.5
        dets = sorted(
            (d for d in self.detections
             if d.results and d.results[0].pose.pose.position.z < NEAR),
            key=lambda d: d.results[0].pose.pose.position.z,
        )[:2]
        count = len(dets)
        section = []

        print(f"\nFiltered detections count: {count}")

        if count == 1:
            closeBlock = []
            color = dets[0].results[0].hypothesis.class_id

            if color == 'red_pillar':
                colorScaler = 1
            elif color == 'green_pillar':
                colorScaler = -1
            else:
                colorScaler = 0

            closeBlock.append(colorScaler)

            front_index = int((0.0 - self.latest_scan.angle_min) / self.latest_scan.angle_increment)
            if 0 <= front_index < len(self.latest_scan.ranges):
                front_wall = self.latest_scan.ranges[front_index]
            else:
                front_wall = None

            print(f"front_wall: {front_wall}")

            yPose = None
            for i in range(3):
                last_dist = None
                print(f"\nScanning section {i}")

                for j in range(self.OFFSET_ANGLE_NUM):
                    angle = i * THETA_1 + j * (THETA_1 / self.OFFSET_ANGLE_NUM)
                    dist = self.range_at_angle(angle)

                    print(f"  j={j}, angle={angle:.3f}, dist={dist}, last_dist={last_dist}")

                    if last_dist is not None and abs(dist - last_dist) > DIP_THRESHOLD:
                        yPose = i
                        print(f"  -> DIP detected in section {i}")
                        break

                    last_dist = dist

                if yPose is not None:
                    break

            closeBlock.append(yPose)
            section.append(closeBlock)
            self.blockPositionsArray.append(section)

            print("\nRESULTS:")
            print(f"closeBlock = {closeBlock}")
            print(f"section = {section}")
            print(f"blockPositionsArray = {self.blockPositionsArray}")

            if yPose is not None:
                if colorScaler == 1:
                    side = "right"
                elif colorScaler == -1:
                    side = "left"
                else:
                    side = "unknown"

                print(f"Waypoint idea: go to section {yPose} and pass on the {side}.")
            else:
                print("Waypoint idea: no section found.")
        else:
            print("This quick tester only demonstrates the single-block branch.")


def main():
    robot = TestRobot()

    color = input("Enter block color (red_pillar / green_pillar): ").strip()
    z = float(input("Enter block z distance (< 1.5 to be detected): ").strip())
    robot.detections.append(Detection(color, z))

    fake_range = int(input("Which range/section should the block be in? (1, 2, or 3): ").strip())
    robot.set_fake_range_section(fake_range)

    robot.addBlocks()


if __name__ == "__main__":
    main()
    