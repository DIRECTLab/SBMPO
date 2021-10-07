import json
import matplotlib.pyplot as plt


#create a dictionary of the states
#TODO needs to be fixed so that it can do just more than example=kinematic
def create_state_dict(path):
    with open(path) as f:
      data = json.load(f)
    #shape the size of the state_dict based on how many states are present
    state_dict = {i:[] for i in range(len(data['trajectory'][0]['state']))}

    #get the state
    for control_state in data['trajectory']:
        state_dict[0].append(control_state['state'][0])
        state_dict[1].append(control_state['state'][1])
        #for key in list(state_dict.keys()):
        #    state_dict[key].append(control_state['state'][key])

    return state_dict


#TODO needs to be fixed so that it can do just more than example=kinematic
def create_state_dict_visualizer(path):
    with open(path) as f:
      data = json.load(f)

    state_dict = {i:[] for i in range(len(data['iterations'][0]['state']))}

    for control_state in data['iterations']:
        for key in list(state_dict.keys()):
            state_dict[key].append(control_state['state'][key])

    #for control_state in data['iterations']:
    #        state_dict['state0'].append(control_state['state'][0])
    #        state_dict['state1'].append(control_state['state'][1])
    #        state_dict['state2'].append(control_state['state'][2])
    #        state_dict['state3'].append(control_state['state'][3])
    #        state_dict['state4'].append(control_state['state'][4])
    #        state_dict['state5'].append(control_state['state'][5])

    return state_dict

def basic_scatter(x, y, gpu=False):
    plt.scatter(x, y, alpha=.25)
    plt.show()

    #TODO Vispy plotting if we need it 

path = "../results/visualizer.json" 
state_dict = create_state_dict_visualizer(path)
print(len(state_dict[0]))
basic_scatter(state_dict[0], state_dict[0])

#use the results folder instead of visualizer
path = "../results/results.json"
state_dict = create_state_dict(path)
print(len(state_dict[0]))
basic_scatter(state_dict[0], state_dict[0])


