import json, math, random
import matplotlib.pyplot as plt

RESOLUTION = 2 # size of a single grid square in meters

def main():
    # load in lidar data
    f = open("fake_lidar_data.json")
    data = json.load(f)
    drone_positions = data['drone_positions']
    lidar_data = data['lidar_data']

    # convert lidar data to global frame using drone position data
    global_lidar_data = [[],[],[]]
    for k in range(len(drone_positions)):
        drone_position = drone_positions[k]
        drone_x = drone_position["x"]
        drone_y = drone_position["y"]
        drone_z = drone_position["z"]
        data_at_position = lidar_data[k]

        for n in range(len(data_at_position)):
            h = data_at_position[n]["h"]
            theta = data_at_position[n]["theta"]*math.pi/180

            x = drone_x
            y = drone_y+h*math.sin(theta)
            z = drone_z-h*math.cos(theta)

            global_lidar_data[0].append(x)
            global_lidar_data[1].append(y)
            global_lidar_data[2].append(z)


    # plot transformed points
    ax = plt.axes(projection='3d')
    zdata = global_lidar_data[2]
    xdata = global_lidar_data[0]
    ydata = global_lidar_data[1]
    ax.scatter3D(xdata, ydata, zdata)
    plt.show()

    # TODO: divide data into grid squares
    # TODO: for each square, remove statistical outliers
    # TODO: average remaining values in the grid square, and save result to height map


def generate_fake_lidar_data():
    data = {"drone_positions": [], "lidar_data": []}

    for x in range(25):
        for y in range(5):
            data["drone_positions"].append({"x": x*0.2, "y": y, "z": 4})

    for k in range(len(data["drone_positions"])):
        data_at_pos = [
            {"theta": -15, "h": 3+random.randint(0,5)*0.1+k*0.003},
            {"theta":   0, "h": 3+random.randint(0,5)*0.1+k*0.003},
            {"theta":  15, "h": 3+random.randint(0,5)*0.1+k*0.003}
        ]
        data["lidar_data"].append(data_at_pos)

    json_object = json.dumps(data, indent=4)
    with open("fake_lidar_data.json", "w") as outfile:
        outfile.write(json_object)


if __name__ == "__main__":
    # generate_fake_lidar_data()
    main()