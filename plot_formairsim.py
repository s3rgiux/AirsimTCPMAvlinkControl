import matplotlib.pyplot as plt

# Function to extract and plot multiple elements
def plot_elements(file_name, element_names):
    data = {element: [] for element in element_names}
    
    with open(file_name, 'r') as file:
        for line in file:
            for element_name in element_names:
                if element_name in line:
                    parts = line.split(', ')
                    for part in parts:
                        if part.startswith(element_name + '='):
                            # Extract the value of the element
                            value = float(part.split('=')[1])
                            data[element_name].append(value)
    
    # Create and customize the plot
    plt.figure(figsize=(8, 6))
    for element_name in element_names:
        plt.plot(data[element_name], label=element_name)
    plt.xlabel('Sample Number')
    plt.ylabel('Element Data')
    plt.title('Plot of Multiple Elements')
    plt.legend()
    plt.grid(True)
    plt.show()

# Specify the file name and the elements to plot
# file_name = 'imu_data.txt'  # Replace with your file name
# elements_to_plot = ['XGyro', 'YGyro', 'ZGyro']

file_name = 'ahrs_data.txt'  # Replace with your file name
elements_to_plot = ['Roll', 'Pitch', 'Yaw']


# Call the function to plot multiple elements
plot_elements(file_name, elements_to_plot)