import csv
import matplotlib.pyplot as plt
prm = []
bezier = []
with open('./total.csv', 'r') as f:
    reader = csv.reader(f)
    for i in reader:
        prm.append(int(i[0]))
        bezier.append(int(i[1]))

x = []
y = []
for i in range(41):
    x.append(i)
    y.append(sum(bezier[i*100:(i+1)*100])/100)
plt.plot(x,y)
plt.show()
    