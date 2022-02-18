import csv
import unicodedata
import re
import os
import sys, getopt

def csv_format_cleaner(filepath, name, output_folder): 
    """
    Prend en entrée le path d'un csv et le nom du csv.
    Créer un nouveau csv contenant uniquement les header et la data formattés correctement.
    """
    
    input = open(filepath)
    reader=csv.reader(input)
    output = open(output_folder + '/' + name + '_formated.csv', 'w', newline='')
    
    for row in reader:
        row =','.join([str(item) for item in row])
        if row.startswith('# Temps'):
            header = ''.join((c for c in unicodedata.normalize('NFD', row) if unicodedata.category(c) != 'Mn'))
            header = re.sub(r'\([^()]*\)', '', header)
            header = header.replace(' ,', ',')
            header=header[2:]
            output.write(header)
            output.write("\n")
        if not row.startswith('#'):
            output.write(row)
            output.write("\n")

def create_folder(folder_name):   
    """
    Crée le folder où seront stockés les csv formatté si il n'existe pas.
    """
    
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    
def format_folder(input_folder, output_folder):
    """
    Parcourt le dossier avec les csv raw, les formatte et les stocke.
    """
    
    for path in os.listdir(input_folder):
        full_path = os.path.join(input_folder, path)
        name = path.split('.')[-2]
        csv_format_cleaner(full_path,name, output_folder)
        
input_folder = "data/log_csv"
output_folder = "data/formated_csv"

"""
commande pour formatter le contenu du dossier log_csv : python src/data_cleaning/csv_formating.py (a la root du projet)
"""

create_folder(output_folder)

format_folder(input_folder, output_folder)