from tkinter.filedialog import askopenfilenames
from cluster_file_utils import clusters_to_csv

file_list = askopenfilenames(filetypes=[("Binary cluster files", "*.clusters"), ])

clusters_to_csv(file_list)