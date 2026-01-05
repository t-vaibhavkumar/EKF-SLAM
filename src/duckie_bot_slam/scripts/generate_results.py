import matplotlib.pyplot as plt
import csv
import os

def generate_report_plots(data_file):
    time, odom_x, odom_y, ekf_x, ekf_y, v_cmd, w_cmd = [], [], [], [], [], [], []

    print(f"Reading data from: {os.path.abspath(data_file)}")
    
    with open(data_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time.append(float(row['time']))
            odom_x.append(float(row['odom_x']))
            odom_y.append(float(row['odom_y']))
            ekf_x.append(float(row['ekf_x']))
            ekf_y.append(float(row['ekf_y']))
            v_cmd.append(float(row['v_cmd']))
            w_cmd.append(float(row['w_cmd']))

    # 1. Trajectory Plot
    plt.figure(figsize=(10, 8))
    plt.plot(odom_x, odom_y, 'r--', label='Odometry (Noisy)')
    plt.plot(ekf_x, ekf_y, 'g-', label='EKF-SLAM (Filtered)')
    
    goals = {'G1': (2,2), 'G2': (8,4), 'G3': (5,7)}
    for name, pos in goals.items():
        plt.scatter(pos[0], pos[1], marker='X', color='blue', s=100)
        plt.text(pos[0], pos[1], f' {name}')
        
    plt.title("Trajectory Comparison: Raw vs SLAM")
    plt.xlabel("X (m)"); plt.ylabel("Y (m)")
    plt.legend(); plt.grid(True)
    
    # SAVE BEFORE SHOW
    save_path1 = os.path.join(os.getcwd(), 'trajectory_plot.png')
    plt.savefig(save_path1)
    print(f"Saved: {save_path1}")

    # 2. Control Signals Plot
    plt.figure(figsize=(10, 4))
    plt.plot(time, v_cmd, label='Linear (v_c)')
    plt.plot(time, w_cmd, label='Angular (w_c)')
    plt.title("Control Signals Over Time")
    plt.legend(); plt.grid(True)
    
    save_path2 = os.path.join(os.getcwd(), 'control_signals.png')
    plt.savefig(save_path2)
    print(f"Saved: {save_path2}")

if __name__ == "__main__":
    generate_report_plots('simulation_data.csv')