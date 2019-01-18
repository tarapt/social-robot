import numpy as np 

f = open("experiment_count.txt", "r")
experiment_number = int(f.read())

f = open("experiment_count.txt", "w")
f.write(str(experiment_number + 1))
