


import yaml
def load_world(file_name):
    with open(file_name) as f:
        world = yaml.safe_load(f)
    
    return world








