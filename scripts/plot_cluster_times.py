from cluster_file_utils import ClusterFileIterator, num_clusters
from tkinter.filedialog import askopenfilenames

import numpy as np
import matplotlib.pyplot as plt

files = askopenfilenames(filetypes=[("Binary cluster files", "*.clusters"), ])

for file in files:
    count = num_clusters(file)
    times = np.empty((count,1))

    for ix, cluster in enumerate(ClusterFileIterator(file)):
        times[ix] = cluster[2]*1.5625e-9

    plt.plot(times)

plt.show()