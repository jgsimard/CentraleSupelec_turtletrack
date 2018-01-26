filename = "position.txt"
step = 2
minimum = 0
maximum = 12
print("filename = ", filename)
print("step =", step, "minimum =", minimum, "maximum =", maximum)
file = open(filename,'w')
for i in range(minimum, maximum, step):
    for j in range(minimum, maximum, step):
        line = str(i) + ".0 " + str(j) + ".0\n"
        file.write(line) 
file.close() 
