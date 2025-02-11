import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import uniform_filter1d

def plot_monitor_data(file_path):
    # Load the monitor CSV file
    data = pd.read_csv(file_path, skiprows=1)  # Skip the first row (commented metadata)

    # Extract rewards and episode lengths (steps)
    rewards = data['r']
    steps = data['l']

    # Calculate the epochs
    epochs = np.arange(len(rewards))

    # filter for episode lengths
    X = epochs.reshape(-1, 1)  # Reshape for sklearn
    y_epF = uniform_filter1d(steps,50)
    
    # filter for rewards
    y_rewards_f = uniform_filter1d(rewards,50)
    
    # Create the subplots
    fig, axs = plt.subplots(2, 2, figsize=(10, 8), sharex=True)

    # Plot rewards
    axs[0, 0].scatter(epochs, rewards, label='Rewards', color='blue')
    axs[0, 0].plot(epochs, y_rewards_f, label='Moving Average 50', color='red', linewidth=2)
    axs[0, 0].set_title('Rewards per Epoch')
    axs[0, 0].set_ylabel('Rewards')
    axs[0, 0].legend(loc='upper left', bbox_to_anchor=(1, 1))
    axs[0, 0].grid()

    # Plot steps (episode lengths) with linear regression
    axs[1, 0].scatter(epochs, steps, label='Steps', color='blue', alpha=0.7)
    axs[1, 0].plot(epochs, y_epF, label='moving Average  50 ', color='red', linewidth=2)
    axs[1, 0].set_title('Steps per Epoch')
    axs[1, 0].set_xlabel('Epochs')
    axs[1, 0].set_ylabel('Steps')
    axs[1, 0].legend(loc='upper left', bbox_to_anchor=(1, 1))
    axs[1, 0].grid()
    
    
    #stackplot
    window_size = 100  
    crash_count = np.convolve(rewards <= -300, np.ones(window_size, dtype=int), mode='same')
    success_count = np.convolve(rewards > 0, np.ones(window_size, dtype=int), mode='same')
    overstepped_count = np.convolve((-300 < rewards) & (rewards < 0), np.ones(window_size, dtype=int), mode='same')    
    axs[0,1].stackplot(epochs,crash_count,overstepped_count,success_count,labels=['Crash', 'Max Step', 'Success'], colors = ['lightpink','lightyellow','lightgreen'])
    # axs[0, 1].plot(crash_count, label='crash', color='red', linewidth=2)
    # axs[0, 1].plot(success_count, label='success', color='green', linewidth=2)
    # axs[0, 1].plot(overstepped_count, label='too many steps', color='blue', linewidth=2)
    axs[0, 1].set_title('crashes & successes per 100 epochs')
    axs[0, 1].set_xlabel('Epochs')
    axs[0, 1].set_ylabel('Steps')
    axs[0, 1].legend(loc='upper left', bbox_to_anchor=(1, 1))
    axs[0, 1].grid()
    
    filtered_steps = steps[rewards > 0]
    filtered_steps_average = uniform_filter1d(filtered_steps,50)
    axs[1, 1].scatter(epochs[:len(filtered_steps)], filtered_steps, label='number of steps', color='blue', alpha = 0.5)
    axs[1, 1].plot(epochs[:len(filtered_steps)], filtered_steps_average, label='moving average 50', color='red', linewidth=2)
    axs[1, 1].set_title('Steps per successed Epoch')
    axs[1, 1].set_xlabel('Epochs')
    axs[1, 1].set_ylabel('Steps')
    axs[1, 1].legend(loc='upper left', bbox_to_anchor=(1, 1))
    axs[1, 1].grid()    
       
    # Adjust layout and show the plot
    plt.tight_layout()
    plt.show()    
    print("average stepnumber", np.sum(steps)/epochs[-1])

# Example usage
plot_monitor_data('/home/cecily/MasterThesis_Cy/MT_Project/controllers/RL_Controller/monitor_logs/env_01.monitor.csv')








