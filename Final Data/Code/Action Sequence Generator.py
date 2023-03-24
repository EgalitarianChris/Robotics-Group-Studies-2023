import csv
import numpy as np

def Log(log,filename="AS.csv"):
    print("saving data")
    
    with open(filename,"w") as csv_file:
        csv_writer = csv.writer(csv_file)
        for row in log:
            csv_writer.writerow(row)
            
    print("saved data to:",filename)

def generate_AS():
    knee_frequency = 0.390
    hip_frequency = 0.390
    AS = np.empty((300,4))
    AS[:,1] = 0.2
    AS[:,3] = 0.2
    AS[:,0] = -1 * np.cos(np.linspace(0,np.pi*2*30*knee_frequency,300))
    AS[:,2] = np.cos(np.linspace(0,np.pi*2*30*hip_frequency,300))
    
    return AS

def main():
    AS = generate_AS()
    for a in AS: 
        print(a)
    Log(AS)
    
main()