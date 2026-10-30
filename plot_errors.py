import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename, ideal = None):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(1,2, figsize=(14,6))


    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].set_title("state space")
    if ideal:
        _, ivals = FileReader(ideal).read_file()
        ix = [p[0] for p in ivals]
        iy = [p[1] for p in ivals]
        style = "rx" if len(ix) == 1 else "r--"
        axes[0].plot(ix, iy, style, label="ideal")
        axes[0].legend()
    axes[0].grid()

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    axes[1].legend()
    axes[1].grid()


    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    parser.add_argument('--ideal', default=None, help='planned path csv')
    args = parser.parse_args()

    print("plotting the files", args.files)

    filenames = args.files
    for filename in filenames:
        plot_errors(filename, args.ideal)



