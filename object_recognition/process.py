import os

def list_files_in_directory(directory, output_file):
    with open(output_file, 'w') as f:
        for root, dirs, files in os.walk(directory):
            for file in files:
                if file.lower().endswith('.jpg'):
                    full_path = os.path.join(root, file)
                    f.write(full_path + '\n')

output_text_file = 'train.txt'
current_dir = os.path.dirname(os.path.abspath(__file__))
list_files_in_directory(current_dir, output_text_file)
