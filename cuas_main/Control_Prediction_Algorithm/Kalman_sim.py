import matplotlib.pyplot as plt


# Specify the radii of the circles
radius_blue = 0.5  # Change this value for the blue circle
radius_red = 0.5  # Change this value for the red circle

# Predefined rouge drone path
dronePath = [(1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6)]

# Function to draw circles
def draw_circles(radius_blue, radius_red):
    fig, ax = plt.subplots()
    fig.patch.set_facecolor('black')  # Set the figure background to black
    ax.set_facecolor('black')          # Set the axes background to black

    # Create circles
    discoveryDronePos = (8, 1)
    rougeDronePos = (1, 1)
    circle_blue = plt.Circle(discoveryDronePos, radius_blue, color='blue', alpha=0.6, label='Discovery Drone')
    circle_red = plt.Circle(rougeDronePos, radius_red, color='red', alpha=0.6, label='Rouge Drone')

    ax.add_artist(circle_blue)
    ax.add_artist(circle_red)

    # Set limits and aspect
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal', adjustable='box')
    
    # Add legend with white font
    legend = ax.legend(loc='upper right', facecolor='black', fontsize=10, framealpha=0.7)
    for text in legend.get_texts():
        text.set_color('white')  # Set legend text color to white

    discoveryDroneStr = 'Discovery Drone Position:' + str(discoveryDronePos)
    rougeDroneStr = 'Rouge Drone Position:' + str(rougeDronePos)

    ax.text(9, 8, discoveryDroneStr, color='white', fontsize=12, ha='center')
    ax.text(9, 7.5, rougeDroneStr, color='white', fontsize=12, ha='center')
    
    plt.axis('off')  # Turn off the axis
    user_input = input("Press any key and then hit Enter: ")
    print(f"You pressed: {user_input}")
    plt.show()
    user_input = input("Press any key and then hit Enter: ")
    print(f"You pressed: {user_input}")

# Draw the circles



draw_circles(radius_blue, radius_red)


