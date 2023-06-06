
import csv

## for reading the file
for id in range(1,2):
    with open('./dataset/task/%d.csv'%id, 'r') as f:
        reader_task = csv.reader(f)
        task = list(reader_task)
        # print(task)
        # print('hello')
        # v = [float(i) for i in task[2]]
        # print(v[1])
    with open('./dataset/map/%d.csv'%id, 'r') as f:
        reader_map = csv.reader(f)
        map = list(reader_map)
        # print(map)
    with open('./dataset/traj/%d.csv'%id, 'r') as f:
        reader_traj = csv.reader(f)
        traj = list(reader_traj)
        # print(traj)


