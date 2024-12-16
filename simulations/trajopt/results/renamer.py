import os

# Specify the directory path where the files are located
directory = '/home/zach/git-repos/Dojo_simulators/echino-sim/trajopt/results/param_sweep/'

# Get the list of files in the directory
files = os.listdir(directory)

for filename in files:
    if filename.endswith('.jld2'):
        # Split the filename by '_' to extract the number 'i'
        parts = filename.split('-')

        parts = parts[0].split('_')
        
        i = parts[-1]
        # Generate the new filename in the format: i.jld2
        new_filename = f'{i}.jld2'
        
        # Construct the full file paths
        old_filepath = os.path.join(directory, filename)
        new_filepath = os.path.join(directory, new_filename)
        
        # Rename the file
        os.rename(old_filepath, new_filepath)
        
        print(f'Renamed "{filename}" to "{new_filename}"')