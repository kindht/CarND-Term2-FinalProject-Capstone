import os

classes_list = ['RED', 'YELLOW', 'GREEN', 'UNKOWN']

#print('Converting classes list to labelmap file:')

with open('test_label_map.pbtxt', mode='w') as f:
    count = 1
    for name in classes_list:
        f.write('item {\n')
        if count == 0:
            f.write('   name: "none_of_the_above"\n')
            f.write('   label: 0\n')
            f.write('}\n')
        else:
            f.write('   id: '+ str(count)+'\n')
            f.write('   name: "'+classes_list[count-1]+'"\n')
            f.write('}\n')
        count = count + 1

#print('Saved labelmap file: test_label_map.pbtxt')

