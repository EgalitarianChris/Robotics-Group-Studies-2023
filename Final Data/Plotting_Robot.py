import csv
import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import os

def load_data(filename):
    with open(filename, newline="") as csv_file:
        reader = csv.reader(csv_file)
        length = sum(1 for _ in reader)
        csv_file.seek(0)
        data = np.empty((length,17))

        for i, row in enumerate(reader):
            data[i] = row

    return data

def plot(file_loc, filename, data_x, data_y):
    fig, (ax1) = plt.subplots(1, sharex=True)
    plt.suptitle(data_y[1] + " vs " + data_x[1])
    ax1.plot(data_x[0], data_y[0], label="filename", linewidth=0.8)
    plt.gcf().text(0, 0, filename, color='grey', fontsize=8)
    plt.xlabel(data_x[2])
    plt.ylabel(data_y[2])
    plt.savefig(file_loc + filename + "/" + filename + "." + data_x[3] + "." + data_y[3] + ".png", bbox_inches="tight", dpi=1200)
    plt.close(fig)

def plot_and_compare(file_loc, filename, data_x, data_y, data_z):
    fig, (ax1) = plt.subplots(1, sharex=True)
    plt.suptitle(data_y[1] + " and " + data_z[1] + " vs " + data_x[1])
    plt.gcf().text(0, 0, filename, color='grey', fontsize=8)
    ax1.plot(data_x[0], data_y[0], label=data_y[1], linewidth=0.8)
    ax1.plot(data_x[0], data_z[0], 'c', label=data_z[1], linewidth=0.8)
    plt.legend()
    plt.xlabel(data_x[2])
    plt.ylabel(data_y[2] + " and " + data_z[2], fontsize=9)
    plt.savefig(file_loc + filename + "/" + filename + "." + data_x[3] + "." + data_y[3] + "." + data_z[3] +".png", bbox_inches="tight", dpi=1200)
    plt.close(fig)

def plot_2_subplots(file_loc, filename, data_x, data_y, data_z):
    fig, (ax1, ax2) = plt.subplots(2, sharex=True)
    plt.suptitle(data_y[1] + " and " + data_z[1] + " vs " + data_x[1])
    plt.gcf().text(0, 0, filename, color='grey', fontsize=8)
    ax1.plot(data_x[0], data_y[0], label=data_y[1], linewidth=0.8)
    ax2.plot(data_x[0], data_z[0], 'c', label=data_z[1], linewidth=0.8)
    ax1.set_title(data_y[1])
    ax2.set_title(data_z[1])
    plt.xlabel(data_x[2])
    ax1.set(ylabel=data_y[2])
    ax2.set(ylabel=data_z[2])
    plt.rc('axes', labelsize=9)
    plt.savefig(file_loc + filename + "/" + filename + "." + data_x[3] + "." + data_y[3] + "." + data_z[3] +"2.png", bbox_inches="tight", dpi=1200)
    plt.close(fig)

def dataset(data):
    time = [data[:,0], "Time", "Time, s", "time"]

    # Observations
    top_obs = [data[:,3], "Swing Angle", "Swing Angle, deg", "top_obs"] # [1][0]
    combined_joint_obs = [data[:,4], "Combined Joint Angle", "Combined Joint Angle, deg", "comb_obs"] # [1][1]
    top_velocity_obs = [data[:,7], "Swing Velocity", "Swing Velocity, deg/s", "top_v_obs"] # [1][2]
    combined_joint_velocity_obs = [data[:,8], "Combined Joint Velocity", "Combined Joint Velocity, deg/s", "comb_v_obs"] # [1][3]

    # Joint Observations
    leg_obs = [data[:,1], "Observed Leg Angle", "Leg Angle, deg", "leg_obs"] # [2][0]
    torso_obs = [data[:,2], "Observed Torso Angle", "Torso Angle, deg", "torso_obs"] # [2][1]
    leg_velocity_obs = [data[:,5], "Observed Leg Velocity", "Leg Velocity, deg/s", "leg_v_obs"] # [2][2]
    torso_velocity_obs = [data[:,6], "Observed Torso Velocity", "Torso Velocity, deg/s", "torso_v_obs"] # [2][3]

    # Joint Actions
    """ for leg_act, apply " * ((90--5.271) - (90-67.895))/2 + ((90--5.271) + (90-67.895))/2 - (90-67.895--5.271)" to get 
    in right range, not sure about how these values were previously normalized so can't fix it, prev defined as:
        action[0]*((90--5.271) - (90-67.895))/2 + ((90--5.271) + (90-67.895))/2,
        action[1]*190.3 + 190.3,
        action[2]*((90-29.186) - (90-70.054))/2 + ((90-29.186) + (90-70.054))/2,
        action[3]*190.3 + 190.3])"""

    leg_act = [data[:, 9], "Target Leg Angle", "Target Leg Angle, deg", "leg_act"]  # [3][0]
    torso_act = [data[:, 10], "Target Torso Angle", "Target Torso Angle, deg", "torso_act"]  # [3][1]
    leg_velocity_act = [data[:, 11], "Target Leg Velocity", "Target Leg Velocity, deg/s", "leg_v_act"]  # [3][2]
    torso_velocity_act = [data[:, 12], "Target Torso Velocity", "Target Torso Velocity, deg/s", "torso_v_act"]  # [3][3]

    # Encoders
    enc_1 = [data[:, 13], "Encoder 1 Angle", "Encoder 1 Angle, degrees", "enc_1"]  # [4][0]
    enc_2 = [data[:, 14], "Encoder 2 Angle", "Encoder 2 Angle, degrees", "enc_2"]  # [4][1]
    enc_3 = [data[:, 15], "Encoder 3 Angle", "Encoder 3 Angle, degrees", "enc_3"]  # [4][2]
    enc_4 = [data[:, 16], "Encoder 4 Angle", "Encoder 4 Angle, degrees", "enc_4"]  # [4][3]

    obs = [top_obs, combined_joint_obs, top_velocity_obs, combined_joint_velocity_obs]
    joint_obs = [leg_obs, torso_obs, leg_velocity_obs, torso_velocity_obs]
    joint_act = [leg_act, torso_act, leg_velocity_act, torso_velocity_act]
    encoders = [enc_1, enc_2, enc_3, enc_4]

    return time, obs, joint_obs, joint_act, encoders


def main():
    file = "ML test - Angle_based_erb_aj - 20 degrees - no bar - log"
    file_loc = "C:/Users/tymch/Desktop/Data Files/"
    if not os.path.isdir(file_loc+file): os.makedirs(file_loc + file)
    data = load_data(file_loc + file + ".csv")

    # Plot every variable independently against time
    for term in range(len(dataset(data)[0]-1)):
        print(str(term+1) + "/" + str(len(dataset(data)[1])))
        for item in range(len(dataset(data)[1])):
            plot(file_loc, file, dataset(data)[0], dataset(data)[term+1][item])


    # Plot variables independently
    # plot_and_compare(file_loc, file, dataset(data)[0], dataset(data)[2][1], dataset(data)[3][1])
    # plot_2_subplots(file_loc, file, dataset(data)[0], dataset(data)[2][0], dataset(data)[3][0])

    # Plot and compare different variables against the same variable
    for item in range(len(dataset(data)[1])):
        # plot_and_compare(file_loc, file, dataset(data)[0], dataset(data)[2][item], dataset(data)[3][item])
        plot_2_subplots(file_loc, file, dataset(data)[0], dataset(data)[2][item], dataset(data)[3][item])
main()