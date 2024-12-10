import pandas as pd
import matplotlib.pyplot as plt

def error_plot():
    df = pd.read_csv("~/csci5551_project/pid_data/pid_control_data11.csv")

    time_data = df.iloc[:, 0].to_numpy()
    error_data = df.iloc[:, 3].to_numpy()

    plt.plot(time_data, error_data, label="Distance Error")
    plt.xlabel("Time (s)")
    plt.ylabel("Distance Error")
    plt.title("PID Control Error")
    plt.legend()
    plt.grid(True)
    plt.savefig("distance_error_moon2.png")
    plt.show()


    linear_vel_data = df.iloc[:, 4].to_numpy()
    angular_vel_data = df.iloc[:, 5].to_numpy()

    plt.plot(time_data, linear_vel_data, label="Linear Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Linear Velocity")
    plt.title("Linear Velocity Oscillation")
    plt.legend()
    plt.grid(True)
    plt.savefig("linear_velocity_1.png")
    plt.show()

    plt.plot(time_data, angular_vel_data, label="Angular Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity")
    plt.title("Angular Velocity Oscillation")
    plt.legend()
    plt.grid(True)
    plt.savefig("angular_velocity_1.png")
    plt.show()

if __name__=="__main__":
    error_plot()
