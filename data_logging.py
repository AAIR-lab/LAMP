import Config
import json
from copy import deepcopy
import argparse
import matplotlib.pyplot as plt

def load_data(domain_name,log_number):
    log_path = Config.get_log_path(domain_name,log_number)
    with open(log_path, "r" ) as f:
        data = json.load(f)
        f.close()

    return data

def change_keys_to_int(data):
    for key in data.keys():
        val = data[key]
        data[int(key)] = deepcopy(val)
        del data[key]
    
    return data

def append_data_logs(*args):
    master_log = {}
    for data_log in args:
        data_log = change_keys_to_int(data_log)
        if len(master_log.keys()) == 0:
            master_log = deepcopy(data_log)
        else:
            max_key = max(master_log.keys())
            for key in data_log.keys():
                master_log[max_key+key] = deepcopy(data_log[key])
        
    return master_log

def save_logs(domain_name,data):
    log_path = Config.get_log_path(domain_name,"master")
    dump_object = json.dumps(data)
    with open(log_path,"w") as f:
        f.write(dump_object)
        f.close()

def get_plot_data(master_data,denominator=None):
    demonstration_dict = {}
    demonstration_data_point_count = {}
    for data_point in master_data:
        if int(master_data[data_point]["num demonstrations"]) not in demonstration_dict.keys():
            demonstration_data_point_count[master_data[data_point]["num demonstrations"]] = 0
            demonstration_dict[master_data[data_point]["num demonstrations"]] = []
        demonstration_data_point_count[master_data[data_point]["num demonstrations"]] += 1
        if master_data[data_point]["successful_run"]:
            demonstration_dict[master_data[data_point]["num demonstrations"]].append(master_data[data_point]["successful_run"])
    
    demonstration_success_rate = {}
    if denominator is None:
        denominator = min(demonstration_data_point_count.values())
    
    for demo_val in demonstration_dict.keys():
        demonstration_success_rate[demo_val] = float(sum(demonstration_dict[demo_val][:denominator]))/float(denominator)

    return demonstration_success_rate

def data_plotting(data):
    x = data.keys()
    x.sort()
    y = []

    for k in x:
        y.append(data[k])

    plt.plot(x,y)
    return plt
    
if __name__ == "__main__":
    argParser = argparse.ArgumentParser()        
    argParser.add_argument("-d","--domain_name",help = "name of domain",nargs='*')
    argParser.add_argument("-c","--count",help = "count of log files",nargs="?",default=1)
    argParser.add_argument("-i","--set_interactive", help = "flag to use interactive session", action="store_true")

    args = argParser.parse_args()
    domain_name = args.domain_name[0]
    log_count = int(args.count)
    interactive = args.set_interactive

    log_list = []
    for i in range(log_count):
        log = load_data(domain_name,i+1)
        log_list.append(log)

    master_data = append_data_logs(*log_list)

    if interactive:
        import IPython
        IPython.embed()    
    
    plotData = get_plot_data(master_data)
    p = data_plotting(plotData)
    p.savefig(Config.get_log_directory(Config.DOMAIN_NAME)+"{}_demonstration_count_plot.png".format(Config.DOMAIN_NAME))
    save_logs(domain_name,master_data)
    

    