import json
import matplotlib.pyplot as plt


#create a dictionary of the states
#TODO needs to be fixed so that it can do just more than example=kinematic
def create_state_dict(path):
    with open(path) as f:
      data = json.load(f)

    state_dict = {'state0': [], 'state1': [], 'state2': [], 'state3': [], 'state4': [], 'state5': []} 

    for control_state in data['trajectory']:
            state_dict['state0'].append(control_state['state'][0])
            state_dict['state1'].append(control_state['state'][1])
            state_dict['state2'].append(control_state['state'][2])
            state_dict['state3'].append(control_state['state'][3])
            state_dict['state4'].append(control_state['state'][4])
            state_dict['state5'].append(control_state['state'][5])

    return state_dict


#TODO needs to be fixed so that it can do just more than example=kinematic
def create_state_dict_visualizer(path):
    with open(path) as f:
      data = json.load(f)

    state_dict = {'state0': [], 'state1': [], 'state2': [], 'state3': [], 'state4': [], 'state5': []} 

    for i in data['iterations']:
    #    print(i['state'])
    #print(data['iterations'][3])

    for control_state in data['iterations']:
            state_dict['state0'].append(control_state['state'][0])
            state_dict['state1'].append(control_state['state'][1])
            state_dict['state2'].append(control_state['state'][2])
            state_dict['state3'].append(control_state['state'][3])
            state_dict['state4'].append(control_state['state'][4])
            state_dict['state5'].append(control_state['state'][5])

    return state_dict

def basic_scatter(x, y):
    plt.scatter(x, y, alpha=.25)
    plt.show()

path = "../results/visualizer.json" 
state_dict = create_state_dict_visualizer(path)
print(len(state_dict['state0']))
basic_scatter(state_dict['state0'], state_dict['state1'])

#use the results folder instead of visualizer
#path = "../results/results.json"
#state_dict = create_state_dict(path)
#print(state_dict['state0'])
#basic_scatter(state_dict['state0'], state_dict['state1'])


