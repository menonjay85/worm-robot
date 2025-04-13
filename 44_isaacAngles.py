home = [119,100,130,114,87]
pos1 = [86,133,164,83,82]
pos2 = [121,62,173,155,59]

isaac_home = [0]*len(home)
isaac_angles = [isaac_home]

def isaac(arr1,arr2):
    tmp = []
    for i in range(len(home)):
        tmp.append(arr1[i]-arr2[i])
    isaac_angles.append(tmp)

isaac(pos1,home)
isaac(pos2,pos1)
isaac(pos2,home)


print(isaac_angles)