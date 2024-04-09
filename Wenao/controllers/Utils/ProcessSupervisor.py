class SupervisorData:
    def __init__(self, robotName):
        self.robotName = robotName
        self.data = {
            "msgID": 0,
            "ballPriority": "",
            "ballOwner": "",
            "ballPosition": [0, 0, 0],
            "RedGoalkeeper": [0, 0, 0],
            "RedDefender": [0, 0, 0],
            "RedForwardB": [0, 0, 0],
            "RedForwardA": [0, 0, 0],
            "BlueGoalkeeper": [0, 0, 0],
            "BlueDefender": [0, 0, 0],
            "BlueForwardB": [0, 0, 0],
            "BlueForwardA": [0, 0, 0],
            "GameStatus": 0,
        }

        self.robot_list = [
            "RedGoalkeeper",
            "RedDefender",
            "RedForwardB",
            "RedForwardA",
            "BlueGoalkeeper",
            "BlueDefender",
            "BlueForwardB",
            "BlueForwardA",
        ]

    def updateData(self, receiver):
        data = receiver.getString()

        if isinstance(data, bytes):
            message = data.decode("utf-8")
        else:
            message = data
        receiver.nextPacket()

        # Split the received string into individual values
        values = message.split(",")

        # Extract and process received values
        if values[0] == "1":
            self.data["ballOwner"] = values[1]
            self.data["ballPosition"] = [float(values[i]) for i in range(2, 4)]

            for i, robot in enumerate(self.robot_list):
                self.data[robot] = [
                    float(values[j]) for j in range(4 + i * 2, 6 + i * 2)
                ]
        elif values[0] == "2":
            self.data["GameStatus"] = values[1]

    def getBallData(self):
        return self.data.get("ballPosition")

    def getBallOwner(self):
        ball_owner = "".join([self.data[i].decode("utf-8") for i in range(2, 11)])
        return ball_owner.strip("*")

    def getBallPriority(self):
        return self.data.get(11).decode("utf-8")

    def getSelfPosition(self) -> list:
        """Get the robot coordinate on the field.

        Returns:
            list: x, y coordinates.
        """
        for key, value in self.data.items():
            # Compare search_string with the keys (case-insensitive)
            if self.robotName.lower() == key.lower():
                # Assuming value is a list with [x, y, z] coordinates
                return value[:2]  # Return only the first two elements (x, y)
