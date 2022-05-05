import random
num_of_targets  = 91
targets = [];
count = 0
while(count<num_of_targets):
    x = random.randint(-130,130)/100
    y = random.randint(-130,130)/100
    if ([x,y] not in targets):
        targets.append([x,y])
        count += 1
        
print(targets)
f = open("Targets.txt", "w")
f.write(str(num_of_targets)+"\n")
for i in targets:
    f.write(str(i[0])+","+str(i[1])+"\n")

f.close()
